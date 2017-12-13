/**
 * Created by vzieglmeier on 14.08.16.
 */

var mongoose    = require("mongoose");

var Schema = mongoose.Schema ({
    groupId: {
        type: String, required: true
    },
    runningNumber: {
        type: Number, required: true
    },
    runId: {
        type: String, required: true
    },
    participantPosition: {
        type: Number, required: true
    },
    murphy: {
        type: Number, required: true
    },
    choices: [{
        type: String
    }],
    participantsChoices: [{
        type: Number
    }],
    imageId: {
        type: Number, required: true
    },
    templateId: {
        type: String
    }
});


var Model = mongoose.model("Playlist", Schema);
module.exports.Model = Model;
