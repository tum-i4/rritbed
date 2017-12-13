/**
 * Created by vzieglmeier on 14.08.16.
 */

var mongoose    = require("mongoose");

var Schema = mongoose.Schema ({
    description: {
        type: String, required: true
    },
    participantPosition: {
        type: Number, required: true
    },
    murphy: {
        type: Number, required: true
    }
});

var Model = mongoose.model("Template", Schema);
module.exports.Model = Model;
