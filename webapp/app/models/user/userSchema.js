/**
 * Created by christopherl on 22.05.16.
 */

var mongoose = require('mongoose');
var bcrypt = require('bcrypt-nodejs');

var Schema = mongoose.Schema ({
    username: {
        type: String,
        required: true,
        unique: true
    },
    password: {
        type: String,
        required: true
    },
    firstname: {
        type: String
    },
    lastname: {
        type: String
    },
    displayname: {
        type: String
    },
    birthdate: {
        type: Date,
        required: true,
        default: '01/01/1970'
    },
    age: {
        type: Number, required: true, default: 0
    },
    coins: {
        type: Number, required: true, default: 150
    },
    imagepath: {
        type: String
    }
}, {
    discriminatorKey : 'type'
});

// Hooks

Schema.pre('save', function(next) {
    var user = this;

    // only hash the password if it has been modified (or is new)
    if (!user.isModified('password')) return next();

    bcrypt.genSalt(10, function(err, salt) {
        if (err) return next(err);

        // hash the password using our new salt
        bcrypt.hash(user.password, salt, null, function (err, hash) {
            if (err) return next(err);

            // override the cleartext password with the hashed one
            user.password = hash;
            next();
        });
    });
});

Schema.post('init', function(result) {

    // Update Age
    var todayDate   = new Date();
    var todayYear   = todayDate.getFullYear();
    var todayMonth  = todayDate.getMonth();
    var todayDay    = todayDate.getDate();

    var birthYear   = this.birthdate.getFullYear();
    var birthMonth  = this.birthdate.getMonth();
    var birthDay    = this.birthdate.getDate();

    var age = todayYear - birthYear;

    if (todayMonth < birthMonth - 1)                         { age--; }
    if (birthMonth - 1 == todayMonth && todayDay < birthDay) { age--; }

    this.age = age;
});

Schema.methods.comparePassword = function(candidatePassword, cb) {
    bcrypt.compare(candidatePassword, this.password, function(err, isMatch) {
        if (err) return cb(err);
        cb(null, isMatch);
    });
};

Schema.methods.toJSON = function() {
    var obj = this.toObject();
    delete obj.password;
    return obj
};

var Model = mongoose.model('User', Schema);

// Export
module.exports.Model = Model;
module.exports.Schema = Schema;