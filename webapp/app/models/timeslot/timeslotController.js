/**
 * Created by christopherl on 22.05.16.
 */

var Timeslot = require('./timeslotSchema');

exports.dayTimeArray = createDayTimeArray();

function createDayTimeArray() {
    
    // Create Time slots
    var days    = ["Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"];
    var times   = ["00:00-03:00", "03:00-06:00", "06:00-09:00", "09:00-12:00", "12:00-15:00", "15:00-18:00", "18:00-21:00", "21:00-24:00"];

    var timeslots = [];

    for (var indexDays = 0; indexDays < days.length; indexDays++) {

        var timeslotsDay = [];
        for (var indexTimes = 0; indexTimes < times.length; indexTimes++) {

            var timeslot = new Timeslot.Model({
                day:  indexDays,
                time: indexTimes,
                timeString: times[indexTimes],
                dayString:  days[indexDays],
                formattedString: days[indexDays] + " " + times[indexTimes]
            });
            timeslotsDay.push(timeslot);
        }

        timeslots.push(timeslotsDay);
    }

    return timeslots
};

