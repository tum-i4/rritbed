/**
 * Created by christopherl on 22.05.16.
 */

module.exports = localRoutes;


function localRoutes(passport) {

    var localController = require('./localController');
    var router = require('express').Router();
    var unless = require('express-unless');

    // var mw = passport.authenticate('jwt', {session: false});
    // mw.unless = unless;

    //middleware
    // router.use(mw.unless({method: ['GET', 'OPTIONS']}));

    router.route('/locals/evaluation/:local_id')
        .get(localController.getLocalEvaluation);
    
    router.route('/locals/:local_id')
        .get(localController.getLocal);

    router.route('/locals/city/:city_name')
        .get(localController.getLocalByCity);

    // Top Guides
    router.route('/toplocals/:locals_count/')
        .get(localController.getTopLocalsWithCount);
    router.route('/toplocals/:locals_count/:city_id')
        .get(localController.getTopLocalsWithCountAndCity);
    

    return router;
}