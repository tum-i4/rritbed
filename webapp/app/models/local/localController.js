/**
 * Created by christopherl on 22.05.16.
 */
var Booking = require('../booking/bookingSchema');
var Local = require('./localSchema');
var TimeslotController = require('../timeslot/timeslotController');

// Create endpoint /api/locals/:local_id for GET
exports.getLocal = function(req, res) {

    Local.Model.findById(req.params.local_id).
        populate("timeslots")
        .exec(function (err, local) {
            if (err) {
                console.log("User not exist.");
                res.status(500).send(err);
                return;
            }
            // If local exists
            if (local) {
                local.coins = 0;
                local.localBookings = [];

                res.send(local);
            } else {
                res.status(404).send("Invalid ID");
            }
        });
};

exports.getLocalByCity = function(req, res) {

    Local.Model.find().
        populate("timeslots").
        where('city').equals(req.params.city_name).
        exec(function (err, locals) {
            if (err) {
                res.status(500).send(err);
                return;
            }

            res.json(locals)
        });
};

// Create endpoint /api/toplocals/:locals_count for GET
exports.getTopLocalsWithCount = function(req, res) {

    var limit = parseInt(req.params.locals_count);

    Local.Model.find().
        populate("timeslots").
        limit(limit).
        where('topGuide').equals('true').
        exec(function (err, local) {
            if (err) {
                res.status(500).send(err);
                return;
            }
            
            local.coins         = 0;
            local.localBookings = [];

            res.json(local);
        });

};

// Create endpoint /api/toplocals/:locals_count/:city_id for GET
exports.getTopLocalsWithCountAndCity = function(req, res) {

    var limit = parseInt(req.params.locals_count);

    Local.Model.find().
    populate("timeslots").
    limit(limit).
    where('topGuide').equals('true').
    where('city').equals(req.params.city_id).
    exec(function (err, local) {
        if (err) {
            res.status(500).send(err);
            return;
        }

        local.coins         = 0;
        local.localBookings = [];

        res.json(local);
    });

};

// Create endpoint /api/locals/evaluation/:locals_id for GET
exports.getLocalEvaluation = function(req, res) {

    var localID = req.params.local_id;

    Local.Model.findById(localID)
        .exec(function (err, local) {
            if (err) {
                res.status(500).send(err);
                return;
            }

            if (!local) {
                res.status(404).send("Invalid ID");
                return;
            }

            Booking.Model.find()
                .where('local').equals(localID)
                .exec(function (err, bookings) {

                    var comments        = [];
                    var ratingsCount    = 0;
                    var ratingsSum      = 0;
                    var confirmedCount  = 0;

                    for(var i = 0; i<bookings.length; i++) {
                        var booking = bookings[i];
                        
                        if(booking.assessmentComment) {
                            comments.push(booking.assessmentComment)
                        }

                        if(booking.assessmentValue != -1) {
                            ratingsSum += booking.assessmentValue;
                            ratingsCount++;
                        }

                        if(booking.confirmed == 1) {
                            confirmedCount++;
                        }
                    }

                    res.send({  averageRatings: ratingsCount != 0 ? ratingsSum/ratingsCount : -1,
                                countRatings:   ratingsCount,
                                comments:       comments,
                                countComments:  comments.length,
                                earnedCoins:    confirmedCount * 25
                    });

                });

        });
};