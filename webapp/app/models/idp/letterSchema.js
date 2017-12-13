/**
 * Created by vzieglmeier on 14.08.16.
 */

var mongoose    = require('mongoose');

var Schema = mongoose.Schema ({
    key: {
        type: String, required: true, default: "A" // A-E
    },
    value: {
        type: String, required: true, default: "A" // A-E, plus sometimes "Murphy"
    }
});


var Model = mongoose.model("Letter", Schema);
module.exports.Model = Model;
