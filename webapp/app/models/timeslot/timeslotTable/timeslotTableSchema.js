/**
 * Created by christopherl on 06.06.16.
 */

var mongoose    = require('mongoose');

var Schema = mongoose.Schema ({
    key: {
        type: String
    },
    freeArray: [{
        type: Boolean, required: true, default: false
    }]
});


var Model = mongoose.model('TimeslotTable', Schema);
module.exports.Model = Model;