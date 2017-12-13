/**
 * Created by christopherl on 22.05.16.
 */
var mongoose    = require('mongoose');
var TimeslotTableSchema = require("../timeslot/timeslotTable/timeslotTableSchema.js");
var UserSchema          = require("../user/userSchema.js");

var Schema = mongoose.Schema ({
    topGuide: {
        type: Boolean, required: true, default: false
    },
    city: {
        type: String
    },
    languages: [{
        type: String
    }],
    specialities: [{
        type: String
    }],
    timeslots: [{
        type: mongoose.Schema.ObjectId, ref: 'Timeslot'
    }],
    vacationUntil: {
        type: Date, default: Date.now()
    }
}, {
    discriminatorKey : 'type'
});

var Model = UserSchema.Model.discriminator('Local', Schema);
module.exports.Model = Model;