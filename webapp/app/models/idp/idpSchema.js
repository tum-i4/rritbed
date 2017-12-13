/**
 * Created by vzieglmeier on 14.08.16.
 */

var mongoose    = require("mongoose");

var Schema = mongoose.Schema ({
    letters: [{
        type: String, required: true
    }],
    runPosition: {
        type: Number, default: 0
    },
    participantPosition: {
        type: Number, required: true
    },
    choices: [{
        type: String
    }],
    participantsChoices: [{
        type: Number
    }],
    numberOfParticipants: {
        type: Number, required: true
    },
    murphy: {
        type: Number, default: -1
    },
    // Caution! Predefined choices have to be reversed.
    predefinedChoices: [{
        type: Number
    }],
    imageId: {
        type: Number, required: true
    },
    templateId: {
        type: String
    }
});

var Model = mongoose.model("Idp", Schema);
module.exports.Model = Model;
