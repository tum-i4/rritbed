/**
 * Created by christopherl on 22.05.16.
 */
var mongoose    = require('mongoose');

var Schema = mongoose.Schema ({
    date: {
        type: Date, required: true, default: Date.now()
    },
    confirmed: {
        type: Number, required: true, default: -1
    },
    user: {
        type: mongoose.Schema.ObjectId, ref: 'User'
    },
    local: {
        type: mongoose.Schema.ObjectId, ref: 'Local'
    },
    assessmentValue: { // -1: No evaluation, 0: Negative, 1: Neutral, 2: Positive
        type: Number, required: true, default: -1
    },
    assessmentComment: {
        type: String
    },
    guestsCount: {
        type: Number, require: true, default: 0
    }
});


var Model = mongoose.model('Booking', Schema);
module.exports.Model = Model;