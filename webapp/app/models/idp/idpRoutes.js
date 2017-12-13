/**
 * Created by vzieglmeier on 14.08.16.
 */

module.exports = idpRoutes;

function idpRoutes() {

    var idpController = require('./idpController');
    var dbController = require('../db/dbController');
    var router = require('express').Router();
    var unless = require('express-unless');
    
    // IDP
    router.get("/run/currentId", idpController.getCurrentId);
    router.get("/run/current", idpController.getStatus);
    router.get("/run/:run_id", idpController.getStatusOf);

    router.post("/run/new", idpController.create);
    router.post("/choice/:choice_position/:choice_id", idpController.choice);

    router.post("/master/templates/start-new", idpController.startTemplateList);
    router.post("/master/templates/next", idpController.nextTemplate);
    router.post("/master/choice/:choice_id", idpController.masterChoice);
    router.post("/master/nextTurn", idpController.nextTurn);
    
    router.post("/master/dump-database", dbController.dumpData);
    router.post("/master/reset-database", dbController.resetDatabase);

    return router;
}
