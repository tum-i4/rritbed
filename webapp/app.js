var Config = require('./config/config.dev_local.js'); // TODO Example Project require('./config/config.js');

/**
 * db connect
 */
var mongoose    = require('mongoose');
var dump        = require("./app/dump/dump.js");

var connection = mongoose.connect([Config.db.host, '/', Config.db.name].join(''), {
  user: Config.db.user,
  pass: Config.db.pass
});

mongoose.connection.on('open', function(){
    console.info("[WARM-UP COMPLETE] Database connected. Don't forget to reset if you need to reinitialize data!");
});


/**
 * create application
 */

var express = require('express');
var bodyParser = require('body-parser');
var cors = require('cors');

var app = express();
app.listen(Config.app.port);

console.info("[SERVER RUNNING] Server listening to port %s.", Config.app.port);


/**
 * app setup
 */

app.use(cors());
app.use(bodyParser.json());
app.use(bodyParser.urlencoded({
    extended: true
}));

/*
app.use(function(req, res, next) {
    var data = new Buffer('');
    req.on('data', function (chunk) {
        console.log("Data");
        data = Buffer.concat([data, chunk]);
    });
    req.on('end', function () {
        req.rawBody = data;
        console.log("End");
        next();
    });
});
*/

//passport

var passport = require('passport');
var jwtConfig = require('./passport/jwtConfig');

app.use(passport.initialize());
jwtConfig(passport);


/**
 * routing
 */

// var userRoutes = require("./app/models/user/userRoutes");
// app.use('/', userRoutes(passport));
//
// var cityRoutes = require("./app/models/city/cityRoutes");
// app.use('/api', cityRoutes());
//
// var localRoutes = require("./app/models/local/localRoutes");
// app.use('/api', localRoutes());
//
// var bookingRoutes = require("./app/models/booking/bookingRoutes");
// app.use('/api', bookingRoutes(passport));
//
// var imageRoutes = require("./app/models/image/imageRoutes");
// app.use('/api', imageRoutes(passport));
//
// var specialityRoutes = require("./app/models/speciality/specialityRoutes");
// app.use('/api', specialityRoutes());
//
// var languageRoutes = require("./app/models/language/languageRoutes");
// app.use('/api', languageRoutes());

var idpRoutes = require("./app/models/idp/idpRoutes");
app.use("/api", idpRoutes());

module.exports = app;
