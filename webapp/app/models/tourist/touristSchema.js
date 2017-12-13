/**
 * Created by christopherl on 20.06.16.
 */

var mongoose    = require('mongoose');
var UserSchema          = require("../user/userSchema.js");

var Schema = mongoose.Schema ({

}, {
    discriminatorKey : 'type'
});

var Model = UserSchema.Model.discriminator('Tourist', Schema);
module.exports.Model = Model;