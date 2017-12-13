/**
 * Created by christopherl on 07.06.16.
 */

module.exports = cityRoutes;


function cityRoutes(passport) {

    var imagesController = require('./imageController');
    var router = require('express').Router();

    var mw = passport.authenticate('jwt', {session: false});

    router.route('/images/:image_id')
        .get(imagesController.getImage);

    router.route('/images/')
        .post(mw, imagesController.postImage);

    return router;
}
