/**
 * Created by christopherl on 23.06.16.
 */

module.exports = specialityRoutes;

function specialityRoutes() {

    var specialityController = require('./specialityController');
    var router = require('express').Router();

    router.route('/specialities')
        .get(specialityController.getSpecialities);

    return router;
}