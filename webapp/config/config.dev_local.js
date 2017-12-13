/**
 * Created by vzieglmeier on 14.08.16.
 */

var Config  = {};
Config.db   = {};
Config.app  = {};
Config.auth = {};

Config.db.host = 'localhost:27017';
Config.db.name = 'webapp';

// Use environment defined port or 3000
Config.app.port = process.env.PORT || 5000;

Config.auth.jwtSecret = "very secret secret";

module.exports = Config;