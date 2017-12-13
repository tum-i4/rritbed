/**
 * Created by vzieglmeier on 14.08.16.
 */

var IdpSchema       = require("./idpSchema.js");
var CurrentIdSchema = require("./currentIdSchema.js");
var PlaylistSchema  = require("./playlistSchema.js");
var TemplateSchema  = require("./templateSchema.js");

var MongooseManager = require("../mongooseManager.js");
var idpController   = require("./idpController");
var Config          = require('../../../config/config.dev_local.js');
var dump            = require("./../../dump/dump.js");

var mongoose = require("../../../node_modules/mongoose/lib");
var ObjectId = mongoose.Types.ObjectId;
var fs = require("fs");

// Global variables
var zeroId = new ObjectId("000000000000");

/* Checks to see if the running number is set correctly. List has to be sorted! */
var verifyIntegrity = function (runs) {
    if (runs.length == 0) return false;

    for (var o = 0; o < runs.length; o++)
    {
        if (runs[o].runningNumber != (o + 1))
        {
            console.error("[VerifyIntegrity] Integrity not verifyable. Running number: " + runs[o].runningNumber + ", o: " + o +
                "\nRun:\n" + runs[o]);
            return false;
        }
    }

    return true;
};

exports.dumpData = function(req, res) {
    if (idpController.oneIsUndefined([req.body.savelocation]))
    {
        res.status(400).send("[Dump] Save location missing from body.");
        idpController.debugInvalidBody(req.body);
        return;
    }

    var lastSymbol = req.body.savelocation[req.body.savelocation.length - 1];
    if (lastSymbol != "\\" && lastSymbol != "/")
    {
        res.status(400).send("[Dump] Add a trailing slash to the save location given.");
        return;
    }

    CurrentIdSchema.Model.findById(zeroId, function(err, id) {
        if (err || !id) {
            res.status(400).send(err);
            return;
        }

        var ids = idpController.generateListFromListLikeAndObject(id.pastGroupIds, id.groupId);
        var errors = [];
        var promises = [];

        // Filling the default run in and saving the current run as well.
        IdpSchema.Model.findById(id.currentId, function (error, currentRun) {
            if (error)
            {
                res.status(500).send("[Dump] Error when creating default run: " + error);
                return;
            }

            PlaylistSchema.Model.create({
                groupId: id.groupId,
                runningNumber: id.runningNumber,
                runId: currentRun._id.toString(),
                murphy: currentRun.murphy,
                participantPosition: currentRun.participantPosition,
                choices: currentRun.choices,
                participantsChoices: currentRun.participantsChoices,
                imageId: currentRun.imageId,
                templateId: currentRun.templateId
            }, function (err, saved) {
                if (err || !saved) {
                    res.status(500).send("[Dump] Error when saving current run:\n" + err);
                    return;
                }

                // It is important to set all three fields!
                id.currentId = dump.testObjId;
                id.groupId = -1;
                var idSavePromise = id.save();

                console.log("Groups:");
                for (var l = 0; l < ids.length; l++) {
                    var groupId = ids[l];

                    if (groupId == "-1") {
                        console.log("%s (skipped)",groupId);
                        continue;
                    }
                    else console.log(groupId);

                    // For each group get all runs.
                    promises.push(PlaylistSchema.Model.find({groupId: groupId},
                        function (err, foundRuns) {
                            if (err) {
                                errors.push(err);
                            }
                            else if (!foundRuns || foundRuns.length == 0) {
                                console.log("Error; !foundRuns: %s, foundRuns.length: %s", !foundRuns, foundRuns.length);
                                errors.push("Nothing found for this group.");
                            }

                            // Data is returned by the resolve function and handled below by Promise.all
                        }));
                }

                Promise.all(promises).then(function (values) {
                    var runss = [];
                    for (var m = 0; m < values.length; m++) {
                        if (values[m].length == 0) console.warn("Empty entry at index " + m + " dropped.");
                        else runss.push(values[m]);
                    }

                    // Sort and verify that each running number is set.
                    var savePromises = [];
                    for (var n = 0; n < runss.length; n++) {
                        runss[n].sort(function (a, b) {
                            if (a.runningNumber > b.runningNumber) return 1;
                            if (a.runningNumber < b.runningNumber) return -1;
                            // a must be equal to b
                            return 0;
                        });

                        if (!verifyIntegrity(runss[n])) {
                            var erroneousGroupId = runss[n][0].groupId;
                            console.error("[Dump] Integrity check failed for group " + erroneousGroupId + ". Group data:");
                            for (var e = 0; e < runss[n].length; e++)
                                console.error(runss[n][e].runningNumber + " --------------------\n" + runss[n][e]);

                            res.status(500).send("[Dump] Integrity of list (groupId: " + erroneousGroupId + ") could not " +
                                "be verified. Please review the data manually.");
                        }

                        var sep = "\t";
                        var result = "_id" + sep + "runId" + sep + "groupId" + sep + "runningNumber" + sep + "templateId" + sep +
                            "participantPosition" + sep + "murphy" + sep + "choices" + sep + "participantChoices" + sep + "imageId";
                        for (var p = 0; p < runss[n].length; p++) {
                            var dt = runss[n][p];
                            result += ("\n" + dt._id + sep + dt.runId + sep + dt.groupId + sep + dt.runningNumber + sep + dt.templateId + sep +
                            dt.participantPosition + sep + dt.murphy + sep + dt.choices + sep + dt.participantsChoices + sep + dt.imageId);
                        }

                        var saveErrors = [];
                        savePromises.push(fs.writeFile(req.body.savelocation + runss[n][0].groupId + ".csv", result, function (err) {
                            if (err) {
                                saveErrors.push(err);
                                return;
                            }

                            console.log("File saved.");
                        }));

                        Promise.all(savePromises).then(function (values) {
                            idSavePromise.then(function (val) {
                                // errors and saveErrors contain the errors during the find and save process.
                                res.send({
                                    message: "[Dump] Dump was successful! Find all error messages attached.",
                                    findErrors: errors, saveErrors: saveErrors
                                });
                            });
                        }, function (err) {
                            res.status(500).send("[Dump] Error when saving to file:\n" + err);
                        });
                    }
                }, function (err) {
                    res.status(500).send("[Dump] Error when loading past runs:\n" + err);
                });
            });
        });
    });
};

exports.resetDatabase = function(req, res) {
    if (idpController.oneIsUndefined([req.body.password]))
    {
        res.status(400).send("[Reset] Password not given.");
        return;
    }

    if (req.body.password != "yes")
    {
        res.status(400).send("[Reset] Password is wrong.");
        return;
    }

    console.info("[Reset] Reset of database triggered.");

    mongoose.connection.db.dropDatabase(function(err, result){
        if (err)
        {
            res.status(500).send("[Reset] Error when dropping database, please try again.");
            console.error("[Reset] Error when dropping database:\n" + err);
            return;
        }

        dump.create();
        res.send("[Reset] Database reset successfully.");
    });
};
