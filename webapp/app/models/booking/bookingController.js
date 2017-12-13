/**
 * Created by christopherl on 22.05.16.
 */

var MongooseManager = require("../mongooseManager.js");
var MailController = require('../mail/mailController');

var Booking     = require('./bookingSchema');
var User        = require('../user/userSchema');
var Local       = require('../local/localSchema');

exports.postBooking = function(req, res) {

    var user = req.user;
    var booking = new Booking.Model(req.body);

    if(!booking.user){
        res.status(400).send('user required');
        return;
    }

    if(!booking.local){
        res.status(400).send('local required');
        return;
    }

    if(!booking.guestsCount){
        res.status(400).send('guestsCount required');
        return;
    }

    if(!booking.user.equals(user._id)) {
        res.status(400).send('not authorized');
        return;
    }

    // Pay Coins
    var cost = booking.guestsCount * 25;

    if(user.coins < cost) {
        res.status(403).send('coins not enough');
        return;
    }

    Local.Model.findById(booking.local)
        .exec(function (err, local) {
            if (err) {
                res.status(400).send('invalid local id given');
                return;
            }

            local.coins += cost;
            local.save();

            user.coins -= cost;
            user.save();

            // Saving booking in the local success method, as the booking shall not be added
            // if the coin value hasn't been adjusted before.
            booking.save(function(err, model) {
                if (err) {
                    res.status(500).send(err);
                    return;
                }

                var invitedGuests = req.body.participantsMail;
                console.log(invitedGuests);
                if(!invitedGuests) {
                    res.status(201).json(model);
                    return
                }

                Booking.Model.populate(booking, {path:"user"}, function(err, user) {
                    if (err) { res.status(500).send(err); return; }
                    Booking.Model.populate(booking, {path:"local"}, function(err, user) {
                        if (err) { res.status(500).send(err); return; }

                        for (var i = 0; i < invitedGuests.length; i++) {
                            var invitedGuest = invitedGuests[i];
                            MailController.sendMail(invitedGuest, booking)
                        }

                        res.status(201).json(model);
                    });
                });
            });
        });
};

// Create endpoint /api/booking/booking_id for GET
exports.getBooking = function(req, res) {

    // Use the Beer model to find a specific beer
    Booking.Model.findById(req.params.booking_id, function(err, movie) {
        if (err) {
            res.status(500).send(err);
            return;
        }

        res.json(movie);
    });
};

// Create endpoint /api/myuserbookings/:user_id for GET
exports.getUserBookings = function(req, res) {
    // Use the Beer model to find a specific beer

     Booking.Model.find()
         .where('user').equals(req.user._id)
         .exec (function(err, booking) {
             if (err) {
                 res.status(500).send(err);
                 return;
             }

             res.json(booking);
         });

    console.log(req.user)


};

// Create endpoint /api/mylocalbookings/:local_id for GET
exports.getLocalBookings = function(req, res) {
    // Use the Beer model to find a specific beer
    Booking.Model.find()
        .where('local').equals(req.user._id)
        .exec (function(err, movie) {
            if (err) {
                res.status(500).send(err);
                return;
            }

            res.json(movie);
        });
};

/////////////////
// Confirm & Reject Booking

// Create endpoint /api/mylocalbookings/:local_id for GET
exports.rejectBooking = function(req, res) {
    // Request is rejected by the local.
    var local = req.user;

    Booking.Model.findById(req.params.booking_id, function(err, booking) {
        if (err) {
            res.status(500).send(err);
            return;
        }

        if (!booking.local.equals(req.user._id)) {
            res.status(401).send('Not authorized');
            return
        }

        // Pay Coins Back
        User.Model.findById(booking.user, function(err, user) {
            if (err) {
                res.status(500).send(err);
                return;
            }

            var cost = booking.guestsCount * 25;

            user.coins += cost;
            user.save();
            local.coins -=  cost;
            local.save();

            if (booking.confirmed != 0) {
                booking.confirmed = 0;
                booking.save();

                res.json(booking);

            } else {
                res.status(403).send('Booking already rejected');
            }
        });
    });

};

exports.confirmBooking = function(req, res) {

    Booking.Model.findById(req.params.booking_id, function(err, booking) {
        if (err) {
            res.status(500).send(err);
            return;
        }

        if (!booking.local.equals(req.user._id)) {
            res.status(401).send('Not authorized');
            return
        }

        if (booking.confirmed != 1) {
            booking.confirmed = 1;
            booking.save();

            res.json(booking);
        } else {
            res.status(403).send('Booking already confirmed');
        }
    });
};

exports.assessBooking = function(req, res) {

    if(!req.body.assessmentValue<0){
        res.status(400).send('value required');
        return;
    }
    
    Booking.Model.findById(req.params.booking_id, function(err, booking) {
        if (err) {
            res.status(500).send(err);
            return;
        }

        if (!booking.user.equals(req.user._id)) {
            res.status(401).send('Not authorized');
            return
        }

        booking.assessmentValue     = req.body.assessmentValue;
        booking.assessmentComment   = req.body.assessmentComment;
        booking.save();

        res.json(booking);
    });

};