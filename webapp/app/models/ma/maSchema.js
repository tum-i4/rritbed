/**
 * Created by vzieglmeier on 13.12.17.
 */

var mongoose    = require("mongoose");

var Schema = mongoose.Schema ({
    messages: [{
        type: String, required: true
    }]
});

var Model = mongoose.model("Ma", Schema);
module.exports.Model = Model;
