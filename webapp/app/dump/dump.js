/**
 * Created by vzieglmeier on 14.08.16.
 */

var IdpSchema       = require("../models/idp/idpSchema.js");
var LetterSchema    = require("../models/idp/letterSchema.js");
var CurrentIdSchema = require("../models/idp/currentIdSchema.js");
var PlaylistSchema  = require("../models/idp/playlistSchema.js");
var TemplateSchema  = require("../models/idp/templateSchema.js");
var ImageSchema  = require("../models/idp/imageSchema.js");

var MongooseManager = require("../models/mongooseManager.js");

var mongoose = require('../../node_modules/mongoose/lib');
var ObjectId = mongoose.Types.ObjectId;

// Constructor
function Dump() {
}

var testObjId = new ObjectId("000000999999");
Dump.testObjId = testObjId;
var testGroupId = -1;
Dump.testGroupId = testGroupId;
var murphy = 3;
var letters = MongooseManager.getLetters(murphy);
var participantsChoices = [0, 0, 0, 0];
if (murphy == -1) participantsChoices.push(0);
// Predefined choices are to be reversed
var debugPredefinedChoices = [1, 2, 3, 2].reverse();

var defaultRun = {
    _id: testObjId,
    runPosition: -1,
    participantPosition: 2,
    murphy: murphy,
    letters: letters,
    choices: ["...", "", "", "", ""],
    participantsChoices: participantsChoices,
    numberOfParticipants: participantsChoices.length,
    predefinedChoices: debugPredefinedChoices,
    imageId: 0
};
Dump.defaultRun = defaultRun;

var templateListLTR = [
    11, 31, 41, 71, 61, 72, 51, 21, 73,
    11, 51, 61, 71, 21, 72, 41, 31, 73,
    11, 31, 21, 71, 61, 72, 51, 41];
Dump.templateListLTR = templateListLTR;

// class methods
Dump.create = function() {
    console.info("[Dump] Loading preset data into database.");

    var templateData = [
        { id: "11", participantPos: 0, murphy: 4, description: "Baseline" },
        { id: "12", participantPos: 0, murphy: 3, description: "Baseline" },
        { id: "13", participantPos: 0, murphy: 1, description: "Baseline" },
        { id: "21", participantPos: 4, murphy: 3, description: "Asch" },
        { id: "31", participantPos: 4, murphy: 3, description: "Roboter widerspricht korrekt" },
        { id: "41", participantPos: 4, murphy: 3, description: "Roboter widerspricht falsch" },
        { id: "51", participantPos: 4, murphy: 2, description: "Mensch widerspricht korrekt" },
        { id: "61", participantPos: 4, murphy: 2, description: "Mensch widerspricht falsch" },
        { id: "71", participantPos: 2, murphy: 4, description: "Noise" },
        { id: "72", participantPos: 3, murphy: 0, description: "Noise" },
        { id: "73", participantPos: 1, murphy: 3, description: "Noise" },
        { id: "74", participantPos: 0, murphy: 2, description: "Noise" }
    ];

    // In the image data, the predefined choices are in LTR order for readability reasons.
    var imageData = [
        // Teil 1
        { imageId: "1",  predefinedChoicesLTR: [2,2,3,2], choicesReadable: "RRFR", templateId: "11" },
        { imageId: "2",  predefinedChoicesLTR: [1,1,1,3], choicesReadable: "FFFR", templateId: "31" },
        { imageId: "3",  predefinedChoicesLTR: [2,2,2,3], choicesReadable: "RRRF", templateId: "41" },
        { imageId: "4",  predefinedChoicesLTR: [2,2,1,2], choicesReadable: "RRFR", templateId: "71" },
        { imageId: "5",  predefinedChoicesLTR: [1,1,1,2], choicesReadable: "RRRF", templateId: "61" },
        { imageId: "6",  predefinedChoicesLTR: [1,1,1,2], choicesReadable: "RRRF", templateId: "72" },
        { imageId: "7",  predefinedChoicesLTR: [2,2,2,1], choicesReadable: "FFFR", templateId: "51" },
        { imageId: "8",  predefinedChoicesLTR: [2,2,2,2], choicesReadable: "FFFF", templateId: "21" },
        { imageId: "9",  predefinedChoicesLTR: [3,3,3,3], choicesReadable: "RRRR", templateId: "73" },
        // Teil 2
        { imageId: "10", predefinedChoicesLTR: [1,1,2,1], choicesReadable: "RRFR", templateId: "11" },
        { imageId: "11", predefinedChoicesLTR: [3,3,3,2], choicesReadable: "FFFR", templateId: "51" },
        { imageId: "12", predefinedChoicesLTR: [3,3,3,1], choicesReadable: "RRRF", templateId: "61" },
        { imageId: "13", predefinedChoicesLTR: [1,1,3,1], choicesReadable: "RRFR", templateId: "71" },
        { imageId: "14", predefinedChoicesLTR: [3,3,3,3], choicesReadable: "FFFF", templateId: "21" },
        { imageId: "15", predefinedChoicesLTR: [3,3,3,1], choicesReadable: "RRRF", templateId: "72" },
        { imageId: "16", predefinedChoicesLTR: [1,1,1,2], choicesReadable: "RRRF", templateId: "41" },
        { imageId: "17", predefinedChoicesLTR: [3,3,3,2], choicesReadable: "FFFR", templateId: "31" },
        { imageId: "18", predefinedChoicesLTR: [2,2,2,2], choicesReadable: "RRRR", templateId: "73" },
        // Teil 3
        { imageId: "19", predefinedChoicesLTR: [3,3,1,3], choicesReadable: "RRFR", templateId: "11" },
        { imageId: "20", predefinedChoicesLTR: [2,2,2,1], choicesReadable: "FFFR", templateId: "31" },
        { imageId: "21", predefinedChoicesLTR: [1,1,1,1], choicesReadable: "FFFF", templateId: "21" },
        { imageId: "22", predefinedChoicesLTR: [3,3,2,3], choicesReadable: "RRFR", templateId: "71" },
        { imageId: "23", predefinedChoicesLTR: [2,2,2,3], choicesReadable: "RRRF", templateId: "61" },
        { imageId: "24", predefinedChoicesLTR: [2,2,2,3], choicesReadable: "RRRF", templateId: "72" },
        { imageId: "25", predefinedChoicesLTR: [1,1,1,3], choicesReadable: "FFFR", templateId: "51" },
        { imageId: "26", predefinedChoicesLTR: [3,3,3,1], choicesReadable: "RRRF", templateId: "41" }
    ];

    var allTemplatePromises = [];
    for (var x = 0; x < templateData.length; x++)
    {
        var template = templateData[x];
        var objectId = new ObjectId(MongooseManager.padIdString(template.id));

        allTemplatePromises.push(TemplateSchema.Model.create({
            _id: objectId,
            description: template.description,
            participantPosition: template.participantPos,
            murphy: template.murphy
        }, function (error, templ) {
            if (error) console.log("[Dump] Error when saving template: " + error);
        }));
    }
    Promise.all(allTemplatePromises).then(function (templates) {
        console.info("[Dump] Successfully created %s templates.", templates.length);
    });

    var allImagePromises = [];
    for (var z = 0; z < imageData.length; z++)
    {
        var image = imageData[z];
        var imgObjId = new ObjectId(MongooseManager.padIdString((image.imageId)));

        allImagePromises.push(ImageSchema.Model.create({
            _id: imgObjId,
            imageId: image.imageId,
            predefinedChoicesLTR: image.predefinedChoicesLTR
        }, function (error, img) {
            if (error) console.log("[Dump] Error when saving image data: " + error);
        }));
    }
    Promise.all(allImagePromises).then(function (imgs) {
        console.info("[Dump] Successfully created %s image data objects.", imgs.length)
    });

    IdpSchema.Model.create(defaultRun, function (error, run)
    {
        if (error) console.error("[Dump] Error when loading default run: " + error);
        else console.info("[Dump] Default run loaded.");
    });

    CurrentIdSchema.Model.create({
        _id: new ObjectId("000000000000"),
        currentId: testObjId,
        runningNumber: 1,
        groupId: testGroupId,
        pastGroupIds: []
    });
};

// export the class
module.exports = Dump;