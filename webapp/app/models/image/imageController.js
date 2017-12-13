/**
 * Created by christopherl on 07.06.16.
 */

var path = require('path');
var fs = require('fs');

var imagePath = "../../../public/";

exports.postImage = function(req, res) {

    var imagePath = path.join(__dirname, imagePath, "Ha");

    fs.writeFile(path, req.rawBody, function(err) {
        if(err) {
            res.status(500).send(err);
        } else {
            res.sendStatus(200);
        }
    });


};

// Create endpoint /api/image/:image_id for GET
exports.getImage = function(req, res) {

    var imagePath = path.join(__dirname, imagePath, req.params.image_id);
    res.sendFile(imagePath);
};
