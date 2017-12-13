/**
 * Created by christopherl on 22.05.16.
 */

module.exports = bookingRoutes;


function bookingRoutes(passport) {

    var bookingController = require('./bookingController');
    var router = require('express').Router();
    var unless = require('express-unless');

    var mw = passport.authenticate('jwt', {session: false});

    //middleware
    router.post('/assessbooking/:booking_id', mw, bookingController.assessBooking);

    router.post('/confirmbooking/:booking_id', mw, bookingController.confirmBooking);
    router.post('/rejectbooking/:booking_id', mw, bookingController.rejectBooking);
    
    router.post('/bookings', mw, bookingController.postBooking);

    router.route('/bookings/:booking_id')
        .get(mw, bookingController.getBooking);
    
    router.route('/userbookings/')
        .get(mw, bookingController.getUserBookings);

    router.route('/localbookings/')
        .get(mw, bookingController.getLocalBookings);
    
    return router;
}