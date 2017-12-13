/**
 * Created by vzieglmeier on 14.08.16.
 */

var mongoose    = require('mongoose');

var Schema = mongoose.Schema ({
    currentId: {
        type: String, required: true, default: "000000000000" // Playlist ID
    },
    runningNumber: {
        type: Number, required: true
    },
    groupId: {
        type: String, required: true
    },
    // The template list is reversed, read from RTL.
    templateList: [{
        type: Number
    }],
    pastGroupIds: [{
        type: String
    }]
});


var Model = mongoose.model('CurrentId', Schema);
module.exports.Model = Model;
