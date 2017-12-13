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

var idpRoutes = require("./app/models/idp/idpRoutes");
app.use("/api", idpRoutes());

module.exports = app;
