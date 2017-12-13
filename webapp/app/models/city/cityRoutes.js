/**
 * Created by christopherl on 22.05.16.
 */


module.exports = cityRoutes;


function cityRoutes(passport) {

    var cityController = require('./cityController');
    var router = require('express').Router();

    router.route('/cities')
        .get(cityController.getCities);

    return router;
}
