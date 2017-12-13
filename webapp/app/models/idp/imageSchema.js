/**
 * Created by vzieglmeier on 26.08.16.
 */

var mongoose    = require("mongoose");

var Schema = mongoose.Schema ({
    imageId: {
        type: String, required: true
    },
    // Caution! Predefined choices have to be in LTR order here!
    predefinedChoicesLTR: [{
        type: Number
    }]
});

var Model = mongoose.model("Image", Schema);
module.exports.Model = Model;
