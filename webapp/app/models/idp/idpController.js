/**
 * Created by vzieglmeier on 14.08.16.
 */

var IdpSchema       = require("./idpSchema.js");
var CurrentIdSchema = require("./currentIdSchema.js");
var PlaylistSchema  = require("./playlistSchema.js");
var TemplateSchema  = require("./templateSchema.js");
var ImageSchema  = require("./imageSchema.js");

var MongooseManager = require("../mongooseManager.js");
var dump            = require("./../../dump/dump.js");

var mongoose = require("../../../node_modules/mongoose/lib");
var ObjectId = mongoose.Types.ObjectId;

// Global variables
var zeroId = new ObjectId("000000000000");

var debugInvalidBody = function (object) {
    for (var propName in object)
    {
        if (object.hasOwnProperty(propName))
        {
            console.log("\n----------\n" + propName + "\n----------\n");
            console.log(object[propName] + "\n");
        }
    }
};
exports.debugInvalidBody = debugInvalidBody;

var oneIsUndefined = function (objects) {
    for (var z = 0; z < objects.length; z++)
    {
        if (objects[z] == undefined) return true;
    }

    return false;
};
exports.oneIsUndefined = oneIsUndefined;

var generateListFromListLikeAndObject = function (listlike, object) {
    if (!listlike || listlike.length == 0)
        return [ object ];
    else if (listlike.length == 1)
        return [ listlike[0], object ];
    else
    {
        listlike.push(object);
        return listlike;
    }
};
exports.generateListFromListLikeAndObject = generateListFromListLikeAndObject;

var create = function(req, res) {
    if (oneIsUndefined([req.body.participantPosition, req.body.murphy, req.body.predefinedChoicesLTR, req.body.group, req.body.newId]))
    {
        res.status(400).send("[Create] Invalid body.");
        debugInvalidBody(req.body);
        return;
    }

    if (req.body.participantPosition == req.body.murphy)
    {
        res.status(400).send("[Create] Participant's position can't be the same as murphy's position!");
        return;
    }

    CurrentIdSchema.Model.findById(zeroId, function(err, id) {
        if (err || !id)
        {
            res.status(500).send(err);
            return;
        }

        // Running number is either incremented (same group) or set to 1.
        var newRunningNumber = -1;
        if (req.body.group == id.groupId)
            newRunningNumber = id.runningNumber + 1;
        else
            newRunningNumber = 1;

        // New id contains group, running number and template id
        var newId = new ObjectId(MongooseManager.createIdString([req.body.group, newRunningNumber, req.body.newId]));

        // Verify that a run with this id does not exist yet.
        IdpSchema.Model.findById(newId, function(err, run) {
            if (!(err || !(run)))
            {
                res.status(400).send("[Create] Run id already used.");
                return;
            }

            console.log("[Create] Saving last run and creating new run with id " + newId + ".");

            // [SAVE] Load the old run
            IdpSchema.Model.findById(new ObjectId(id.currentId), function(err, run) {
                if (err || !run)
                {
                    res.status(500).send("[Create] Could not find data of past run while trying to save.");
                    return;
                }

                // [SAVE] Saving the old run
                var errMess = "[Create] Error when saving past run";
                PlaylistSchema.Model.create({
                    groupId: id.groupId,
                    runningNumber: id.runningNumber,
                    runId: run._id.toString(),
                    murphy: run.murphy,
                    participantPosition: run.participantPosition,
                    choices: run.choices,
                    participantsChoices: run.participantsChoices,
                    imageId: run.imageId,
                    templateId: run.templateId
                }, function (err, savedData) {
                    if (err || (savedData == undefined))
                    {
                        res.status(500).send(errMess + ":\n" + err);
                        console.error(errMess + ".\nsavedData:" + savedData + "\nError:\n" + err);
                        return;
                    }

                    // Create new data for id object
                    var newIdData = { currentId: newId, runningNumber: newRunningNumber };
                    // Add group id, if necessary.
                    if (req.body.group != id.groupId)
                    {
                        newIdData.groupId = req.body.group;
                        newIdData.pastGroupIds = generateListFromListLikeAndObject(id.pastGroupIds, id.groupId);
                    }

                    // [CREATE] Save the new run
                    CurrentIdSchema.Model.update({ _id: zeroId }, newIdData, function(err) {
                        if (err)
                        {
                            res.status(500).send(err);
                            console.error("[Create] Error when saving past run:\n" + err);
                            return;
                        }

                        // [CREATE] New run
                        var participantsChoices = [0, 0, 0, 0];
                        var numberOfParticipants = 4;
                        if (req.body.murphy == -1 || req.body.murphy == "-")
                        {
                            participantsChoices.push(0);
                            numberOfParticipants++;
                        }

                        IdpSchema.Model.create({
                            _id:     newId,
                            participantPosition: req.body.participantPosition,
                            letters: MongooseManager.getLetters(req.body.murphy),
                            murphy: req.body.murphy,
                            choices: ["...", "", "", "", ""],
                            participantsChoices: participantsChoices,
                            numberOfParticipants: numberOfParticipants,
                            predefinedChoices: req.body.predefinedChoicesLTR.reverse(),
                            imageId: newRunningNumber,
                            templateId: req.body.templateId
                        }, function (err, saved) {
                            var saveErr = "[Create] Error when saving new run: ";
                            if (err)
                            {
                                res.status(500).send(saveErr + "\n" + err);
                                return;
                            }
                            else if (saved == undefined)
                            {
                                res.status(500).send(saveErr + "Saved data is null or undefined.");
                                return;
                            }

                            res.status(201).send("[Create] Creation was successful. New id: " + newId);
                        });
                    });
                });
            });
        });
    });
};
exports.create = create;

exports.getCurrentId = function(req, res) {
    CurrentIdSchema.Model.findById(zeroId, function(err, id) {
        if (err || !id) {
            res.status(400).send(err);
            return;
        }

        res.send(id.currentId);
    });
};

exports.getStatus = function(req, res) {
    CurrentIdSchema.Model.findById(zeroId, function(err, id) {
        if (err || !id) {
            res.status(500).send(err);
            return;
        }

        req.params.run_id = id.currentId;
        getStatusOf(req, res);
    });
};

var getStatusOf = function(req, res) {
    IdpSchema.Model.findById(new ObjectId(req.params.run_id), function(err, run) {
        if (err || !run) {
            res.status(500).send(err);
            return;
        }

        res.send(run);
    });
};
exports.getStatusOf = getStatusOf;

// ---------------------------------------- //
// ---------------- CHOICE ---------------- //

var advance = function (index) {
    if (index == 4) return -1;
    else return ++index;
};

var participantsDone = function (participantsChoices) {
    for (var v = 0; v < participantsChoices.length; v++)
    {
        if (participantsChoices[v] == 0) return false;
    }

    return true;
};

var internalSetChoiceAdvanceAndFinish = function(req, res, run, choice, advanceOverride, methodName) {
    // Advance position
    if (advanceOverride || participantsDone(run.participantsChoices))
    {
        run.choices.set(run.runPosition, choice);

        // Advance the position. If we have positions left, set the waiting indicator.
        run.runPosition = advance(run.runPosition);
        if (run.runPosition != -1) run.choices.set(run.runPosition, "...");
    }

    run.save(function(err) {
        if (err) {
            res.status(500).send(err);
            console.error("[" + methodName + "] Error when saving run:\n" + err);
            return;
        }

        if (run.runPosition == "-1")
            res.status(202).send("[" + methodName + "] Run done");
        else
            res.send(run);
    });
};

var internalChoice = function(req, res, master){
    CurrentIdSchema.Model.findById(zeroId, function(err, id) {
        if (err || !id) {
            res.status(500).send(err);
            return;
        }

        IdpSchema.Model.findById(new ObjectId(id.currentId), function(err, run) {
            if (err || !run) {
                res.status(500).send(err);
                return;
            }

            // Verify run isn't finished
            if (run.runPosition == "-1")
            {
                res.status(400).send("[Choice] Run already finished - please start a new one!");
                return;
            }

            // Verify valid ID for participants
            if (!master && (req.body.id > run.numberOfParticipants - 1))
            {
                res.status(400).send("[Choice] Only " + run.numberOfParticipants + " participants registered, " +
                    "ID " + req.body.id + " is invalid!");
                return;
            }

            // Verify valid parameters
            if ((req.params.choice_id != "1" && req.params.choice_id != "2" && req.params.choice_id != "3"))
            {
                console.log("Choice id: " + req.params.choice_id);

                res.status(400).send("[Choice] Invalid parameters, please check again.");
                return;
            }

            // Verify valid request
            if (!master)
            {
                if ((run.participantPosition != run.runPosition) || (req.params.choice_position != run.runPosition))
                {
                    console.error("Choice position: " + req.params.choice_position);
                    console.error("Participant position: " + run.participantPosition + ", run position: " + run.runPosition);

                    res.status(400).send("[Choice] Invalid choice - participant was not allowed to vote.");
                    return;
                }

                if (run.participantsChoices[req.body.id] != 0)
                {
                    res.status(400).send("[Choice] Participant ID already in use!");
                    return;
                }

                run.participantsChoices.set(req.body.id, req.params.choice_id);
            }
            else
            {
                if (run.participantPosition == run.runPosition)
                {
                    res.status(400).send("[Choice] Participant position is run position - master can't vote now!");
                    return;
                }
            }

            // Select string for choices array.
            var choice = "";
            if (master) choice = req.params.choice_id;

            internalSetChoiceAdvanceAndFinish(req, res, run, choice, master, "Choice");
        });
    });
};

exports.masterChoice = function(req, res) {
    internalChoice(req, res, true);
};

exports.choice = function(req, res) {
    if (oneIsUndefined([req.body.id]) || req.body.id > 4 || req.body.id < 0)
    {
        res.status(500).send("Invalid id supplied, please choose a number from 0 to 4.");
        return;
    }

    internalChoice(req, res, false);
};

exports.nextTurn = function(req, res) {
    CurrentIdSchema.Model.findById(zeroId, function(err, id) {
        if (err || !id) {
            res.status(500).send(err);
            return;
        }

        IdpSchema.Model.findById(new ObjectId(id.currentId), function (err, run) {
            if (err || !run) {
                res.status(500).send(err);
                return;
            }

            // Verify run isn't finished yet.
            if (run.runPosition == "-1")
            {
                res.status(400).send("[NextTurn] Run is already finished! Please start a new one.");
                return;
            }

            // Verify it's not the participants' choice
            if (run.runPosition == run.participantPosition)
            {
                res.status(400).send("[NextTurn] It's the participants' turn! Please wait until they finished.");
                return;
            }

            // Verify predefined choices exist
            if (!run.predefinedChoices || run.predefinedChoices.length == 0)
            {
                res.status(500).send("[NextTurn] No predefined choices exist - possible data error!");
                return;
            }

            var next = run.predefinedChoices.pop();

            // Verify correctness of value
            if (next != "1" && next != "2" && next != "3")
            {
                res.status(500).send("[NextTurn] Invalid next choice " + next + " - possible data error!");
                return;
            }

            internalSetChoiceAdvanceAndFinish(req, res, run, next, true, "NextTurn");
        });
    });
};

// ------------------------------------------ //
// ---------------- TEMPLATE ---------------- //

var nextTemplate = function(req, res) {
    CurrentIdSchema.Model.findById(zeroId, function(err, id) {
        if (err || !id) {
            res.status(500).send(err);
            return;
        }

        if (!id.templateList)
        {
            res.status(500).send("[NextTemplate] No more templates exist - possible data error!");
            return;
        }

        if (id.templateList.length == 0)
        {
            res.status(400).send("[NextTemplate] No more templates available - try to start a new run.");
            return;
        }

        // Get id of next template and save the changed current id object.
        var nextTemplateIdString = id.templateList.pop().toString();
        var idSavePromise = id.save();

        console.info("[NextTemplate] Loading template with id " + nextTemplateIdString + ".");

        // Create an ObjectId.
        var nextTemplateObjectId = MongooseManager.createObjectIdFromString(nextTemplateIdString);

        TemplateSchema.Model.findById(nextTemplateObjectId, function (err, template) {
            if (err || !template) {
                res.status(500).send("[Template] No template found - did you supply a wrong template id?\n " + err);
                return;
            }

            var nextRunningNumber = id.runningNumber + 1;
            var nextImageObjectId = MongooseManager.createObjectIdFromString(nextRunningNumber.toString());

            ImageSchema.Model.findById(nextImageObjectId, function (err, imageData) {
                if (err || !imageData) {
                    res.status(500).send("[Image] No image data found - is data missing from the DB?\n " + err);
                    return;
                }

                console.info("-> Loaded template " + nextTemplateIdString + " and data for corresponding image " + nextRunningNumber + ".");

                req.body.participantPosition = template.participantPosition;
                req.body.murphy = template.murphy;
                // Predefined choices must be LTR, they are reversed in create()!
                req.body.predefinedChoicesLTR = imageData.predefinedChoicesLTR;
                // If the group id is not set, we keep it.
                if (oneIsUndefined([req.body.group]))
                    req.body.group = id.groupId;
                req.body.newId = nextTemplateIdString;
                req.body.imageId = imageData.imageId;
                req.body.templateId = nextTemplateIdString;

                idSavePromise.then(function(val) {
                    create(req, res)
                });
            });
        });
    });
};
exports.nextTemplate = nextTemplate;

exports.startTemplateList = function(req, res) {
    if (oneIsUndefined([req.body.group]))
    {
        res.status(400).send("[StartTemplateList] Invalid body.");
        debugInvalidBody(req.body);
        return;
    }

    console.log("-----------------------------------------------------------\n" +
        "-----------------------------------------------------------\n" +
        "[StartTemplateList] Restart of template list triggered with group " + req.body.group + ".");
    
    // LTR order, they are reversed when saving.
    var templateListLTR = dump.templateListLTR;

    CurrentIdSchema.Model.findById(zeroId, function(err, id) {
        if (err || !id) {
            res.status(500).send(err);
            return;
        }

        // Check to see if group id has been used before.
        for (var g = 0; g < id.pastGroupIds.length; g++)
        {
            if (id.pastGroupIds[g] == req.body.group || id.groupId == req.body.group)
            {
                res.status(400).send("[StartTemplateList] Group id already used!");
                return;
            }
        }

        // Running number is set in create
        id.templateList = templateListLTR.reverse();
        id.save(function(err) {
            if (err) {
                res.status(500).send(err);
                console.error("[StartTemplateList] Error when starting new template list:\n" + err);
                return;
            }

            nextTemplate(req, res);
        });
    });
};
