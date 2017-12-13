/**
 * Created by christopherl on 22.05.16.
 */

// Create endpoint /api/cities for GET
exports.getCities = function(req, res) {

    var cities = ['Berlin', 'Munich', 'Hamburg', 'Milano', 'Rome', 'Stuttgart'];
    res.json(cities);

};
