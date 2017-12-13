/**
 * Created by vzieglmeier on 13.12.17.
 */

module.exports = maRoutes;

function maRoutes() {

    var maController = require('./maController');
    var dbController = require('../db/dbController');
    var router       = require('express').Router();
    var unless       = require('express-unless');

    // Log
    router.post("/log", maController.log);

    // DB
    
    router.post("/master/dump-database", dbController.dumpData);
    router.post("/master/reset-database", dbController.resetDatabase);

    return router;
}
