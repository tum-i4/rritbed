/**
 * Created by christopherl on 22.05.16.
 */

// Create endpoint /api/cities for GET
exports.getLanguages = function(req, res) {

    var specialities = ['German', 'English', 'Italian', 'Spanish'];
    res.json(specialities);
};