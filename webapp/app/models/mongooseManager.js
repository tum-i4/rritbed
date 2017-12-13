/**
 * Created by vzieglmeier on 14.08.216.
 */

var mongoose = require('../../node_modules/mongoose/lib');
var ObjectId = mongoose.Types.ObjectId;

function MongooseManager() {
}

MongooseManager.getLetters = function (murphy_index) {
    var letters = ["A", "B", "C", "D", "E"];

    if (murphy_index != "-1")
    {
        letters[murphy_index] = "M";
    }

    return letters;
};

MongooseManager.padIdString = function (newId) {
    while (newId.length < 12)
    {
        newId = "0" + newId;
    }

    return newId;
};

MongooseManager.createIdString = function (ids) {
    var idString = "";
    for (var y = 0; y < ids.length; y++)
    {
        idString += ids[y];
    }

    return MongooseManager.padIdString(idString);
};

MongooseManager.createObjectIdFromString = function (newId) {
    return new ObjectId(MongooseManager.padIdString(newId));
};

MongooseManager.createObjectIdFromList = function (ids) {
    return new ObjectId(MongooseManager.createIdString(ids));
};

// export the class
module.exports = MongooseManager;
