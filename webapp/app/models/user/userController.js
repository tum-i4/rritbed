/**
 * Created by christopherl on 22.05.16.
 */

var Config = require('../../../config/config.dev_local.js');
var User = require('./userSchema');
var Tourist = require('../tourist/touristSchema');
var Local = require('../local/localSchema');

var Timeslot            = require('../timeslot/timeslotSchema');
var TimeslotController  = require("../timeslot/timeslotController");

var jwt = require('jwt-simple');

module.exports.login = function(req, res){

    if(!req.body.username){
        res.status(400).send('username required');
        return;
    }
    if(!req.body.password){
        res.status(400).send('password required');
        return;
    }

    User.Model.findOne({username: req.body.username}, function(err, user){
        if (err) {
            res.status(500).send(err);
            return;
        }

        if (!user) {
            res.status(401).send('Invalid Credentials');
            return;
        }
        user.comparePassword(req.body.password, function(err, isMatch) {
            if(!isMatch || err){
                res.status(401).send('Invalid Credentials');
            } else {
                res.status(200).json({token: createToken(user)});
            }
        });
    });

};

module.exports.signup = function(req, res){
    
    if(!req.body.username){
        res.status(400).send('username required');
        return;
    }
    if(!req.body.password){
        res.status(400).send('password required');
        return;
    }

    var user = new User.Model();

    user.username = req.body.username;
    user.password = req.body.password;

    user.save(function(err) {
        if (err) {
            res.status(500).send(err);
            return;
        }

        res.status(201).json({token: createToken(user)});
    });
};

module.exports.unregister = function(req, res) {
    req.user.remove().then(function (user) {
        res.sendStatus(200);
    }, function(err){
        res.status(500).send(err);
    });
};

module.exports.upgrade = function(req, res){

    var user = req.user;

    if (user instanceof Local.Model) {
        res.status(403).send('User already Local');
        return;
    }

    // Update Values
    user.type = "Local";

    // Save User
    user.save(function(err) {
        if (err) {
            res.status(500).send(err);
            return;
        }

        // Reload from Database
        User.Model.findById(user._id)
            .exec(function (err, tourist) {
                if (err) {
                    res.status(500).send(err);
                    return;
                }
                res.send(tourist);
            });
    });
};

module.exports.downgrade = function(req, res){

    var user = req.user;

    if (user instanceof Tourist.Model) {
        res.status(403).send('User already Tourist');
        return;
    }

    // Update Values
    user.type = "Tourist";

    user.languages          = undefined;
    user.localBookings      = undefined;
    user.timeslotsTables    = undefined;
    user.timeslots          = undefined;
    user.specialities       = undefined;
    user.vacationUntil      = undefined;

    // Save User
    user.save(function(err) {
        if (err) {
            res.status(500).send(err);
            return;
        }

        // Reload from Database
        User.Model.findById(user._id)
            .exec(function (err, local) {
                if (err) {
                    res.status(500).send(err);
                    return;
                }
                res.send(local);
            });
    });
};

module.exports.existsEmail = function(req, res) {
    if (!req.body.email)
    {
        res.status(400).send("No email given in body.");
        return;
    }

    User.Model.findOne({username: req.body.email}, function(err, user){
        if (err || !(user)) {
            res.status(204).send();
            return;
        }

        res.status(200).send({email: req.body.email});
    });
};

function createToken(user) {
    var tokenPayload = {
        user: {
            _id: user._id,
            username: user.username
        }

    };
    return jwt.encode(tokenPayload, Config.auth.jwtSecret);
};

// READ UPDATE DELETE

// Create endpoint /api/movies/:movie_id for GET
exports.getUser = function(req, res) {
    
    // Use the Beer model to find a specific beer
    User.Model.findById(req.params.user_id, function(err, user) {
        if (err) {
            res.status(500).send(err)
            return;
        };

        if(user instanceof Local.Model) {
            Local.Model.populate(user, {path:"timeslots"}, function(err, local) {
                if (err) {
                    res.status(500).send(err);
                    return;
                };

                res.json(local);

            });
        } else {
            res.json(user);
        }
    });
};

// Create endpoint /api/movies/:movie_id for PUT
exports.putUser = function(req, res) {

    User.Model.findById(req.params.user_id, function(err, user) {
        if (err) {
            res.status(500).send(err);
            return;
        }

        if(user instanceof Local.Model) {
            user.displayname    = req.body.displayname;
            user.city           = req.body.city;
            user.specialities   = req.body.specialities;
            user.languages      = req.body.languages;

            // Timeslots
            var timeslotsArray = TimeslotController.dayTimeArray;
            var timeslots = [];

            for(var i = 0; i < req.body.timeslots.length; i++) {
                var timeslotFromBody = req.body.timeslots[i];

                if (timeslotFromBody.day >= 0 && timeslotFromBody.day < timeslotsArray.length) {
                    if (timeslotFromBody.time >= 0 && timeslotFromBody.time < timeslotsArray[0].length) {
                        timeslots.push(timeslotsArray[timeslotFromBody.day][timeslotFromBody.time]._id)
                    }
                }
            }
            
            user.timeslots = timeslots;
        }

        user.save(function(err) {
            if (err) {
                res.status(500).send(err);
                return;
            }

            res.json(user);
        });


    });

};

exports.deleteUser = function(req, res) {
    User.Model.findById(req.params.user_id, function(err, user) {
        if (err) {
            res.status(500).send(err);
            return;
        }

        user.remove();
    });
};

exports.postUser = function(req, res) {
    var user = new User.Model();
    user.save()
};