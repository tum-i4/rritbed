/**
 * Created by vzieglmeier on 13.12.17.
 */

var maSchema        = require("./maSchema.js");

var MongooseManager = require("../mongooseManager.js");
var dump            = require("./../../dump/dump.js");

var mongoose = require("../../../node_modules/mongoose/lib");
var ObjectId = mongoose.Types.ObjectId;

// Global variables
var zeroId = new ObjectId("000000000000");

var debugInvalidBody = function (object) {
    for (var propName in object)
    {
        if (object.hasOwnProperty(propName))
        {
            console.log("\n----------\n" + propName + "\n----------\n");
            console.log(object[propName] + "\n");
        }
    }
};
exports.debugInvalidBody = debugInvalidBody;

var oneIsUndefined = function (objects) {
    for (var z = 0; z < objects.length; z++)
    {
        if (objects[z] == undefined) return true;
    }

    return false;
};
exports.oneIsUndefined = oneIsUndefined;

var generateListFromListLikeAndObject = function (listlike, object) {
    if (!listlike || listlike.length == 0)
        return [ object ];
    else if (listlike.length == 1)
        return [ listlike[0], object ];
    else
    {
        listlike.push(object);
        return listlike;
    }
};
exports.generateListFromListLikeAndObject = generateListFromListLikeAndObject;

var log = function(req, res) {
    console.log("TODO TODO TODO TEMP TEMP TEMP NOT IMPLEMENTED");
    throw("Not implemented");
};
exports.log = log;
