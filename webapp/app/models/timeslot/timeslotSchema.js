/**
 * Created by christopherl on 22.05.16.
 */

var mongoose    = require('mongoose');

var Schema = mongoose.Schema ({
    day: {
        type: Number, required: true, default: 0 // 0 Monday <-> 6 Sunday
    },
    time: {
        type: Number, required: true, default: 0 // 0 00:00 - 03:00 <-> 7 21:00 - 24:00
    },
    dayString: {
        type: String
    },
    timeString: {
        type: String
    },
    formattedString: {
        type: String
    }
});


var Model = mongoose.model('Timeslot', Schema);
module.exports.Model = Model;