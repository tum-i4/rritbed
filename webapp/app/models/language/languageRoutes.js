/**
 * Created by christopherl on 22.05.16.
 */


module.exports = languageRoutes;


function languageRoutes() {

    var languageController = require('./languageController');
    var router = require('express').Router();

    router.route('/languages')
        .get(languageController.getLanguages);

    return router;
}
