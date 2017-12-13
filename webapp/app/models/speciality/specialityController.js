


// Create endpoint /api/cities for GET
exports.getSpecialities = function(req, res) {

    var specialities = ['Museum', 'Nightlife', 'Park'];

    res.json(specialities);

};