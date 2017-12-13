module.exports = userRoutes;

function userRoutes(passport) {

    var userController = require('./userController');
    var router = require('express').Router();

    var mw = passport.authenticate('jwt', {session: false});

    // Get & Update
    router.route('/users/:user_id')
        .get(userController.getUser)
        .put(mw, userController.putUser)
        .delete(userController.deleteUser); // Optional Assignment 4

    router.route('/users/')
        .post(userController.postUser); // Optional Assignment 4

    router.post('/users/exists', userController.existsEmail);
    
    router.post('/login', userController.login);
    router.post('/signup', userController.signup);

    router.post('/unregister', mw, userController.unregister);
    router.post('/upgrade', mw, userController.upgrade);
    router.post('/downgrade', mw, userController.downgrade);

    return router;

}