/**
 * Created by christopherl on 03.07.16.
 */

var nodemailer = require('nodemailer');
var dateFormat = require('dateformat');

var BASEURL = 'http://localhost:8080';

exports.sendMail = function(to, booking) {

    // mail
    var transporter = nodemailer.createTransport({
        service:    'Gmail',
        auth:       { user: 'findalocalinfo@gmail.com', pass: 'SEBASEBA' }
    });

    var mailOptions = {
        from:       'findalocalinfo@gmail.com',
        to:         to,
        subject:    'Booking Invitation',

        html: '<!DOCTYPE html PUBLIC "-//W3C//DTD XHTML 1.0 Transitional//EN" "http://www.w3.org/TR/xhtml1/DTD/xhtml1-transitional.dtd">'
                + '<html xmlns="http://www.w3.org/1999/xhtml">'
                + '<head>'
                + '<meta http-equiv="Content-Type" content="text/html; charset=utf-8" />'
                + '<title>A Simple Responsive HTML Email</title>'
                + '<style type="text/css">'
                + 'body {margin: 0; padding: 0; min-width: 100%!important;}'
                + 'img {height: auto;}'
                + '.content {width: 100%; max-width: 600px;}'
                + '.header {padding: 40px 30px 20px 30px;}'
                + '.innerpadding {padding: 30px 30px 30px 30px;}'
                + '.borderbottom {border-bottom: 1px solid #f2eeed;}'
                + '.subhead {font-size: 15px; color: #ffffff; font-family: sans-serif; letter-spacing: 10px;}'
                + '.h1, .h2, .bodycopy {color: #153643; font-family: sans-serif;}'
                + '.h1 {font-size: 33px; line-height: 38px; font-weight: bold;}'
                + '.h2 {padding: 0 0 15px 0; font-size: 24px; line-height: 28px; font-weight: bold;}'
                + '.bodycopy {font-size: 16px; line-height: 22px;}'
                + '.button {text-align: center; font-size: 18px; font-family: sans-serif; font-weight: bold; padding: 0 30px 0 30px;}'
                + '.button a {color: #ffffff; text-decoration: none;}'
                + '.footer {padding: 20px 30px 15px 30px;}'
                + '.footercopy {font-family: sans-serif; font-size: 14px; color: #ffffff;}'
                + '.footercopy a {color: #ffffff; text-decoration: underline;}'
                + '@media only screen and (max-width: 550px), screen and (max-device-width: 550px) {'
                + 'body[yahoo] .hide {display: none!important;}'
                + 'body[yahoo] .buttonwrapper {background-color: transparent!important;}'
                + 'body[yahoo] .button {padding: 0px!important;}'
                + 'body[yahoo] .button a {background-color: #e05443; padding: 15px 15px 13px!important;}'
                + 'body[yahoo] .unsubscribe {display: block; margin-top: 20px; padding: 10px 50px; background: #2f3942;'
                  + ' border-radius: 5px; text-decoration: none!important; font-weight: bold;}'
                + '}'
                + '/*@media only screen and (min-device-width: 601px) {'
                + '.content {width: 600px !important;}'
                + '.col425 {width: 425px!important;}'
                + '.col380 {width: 380px!important;}'
                + '}*/'
                + '</style>'
                + '</head>'
                + '<body yahoo>'
                + '<table width="100%" bgcolor="#f6f8f1" border="0" cellpadding="0" cellspacing="0">'
                + '<tr>'
                + '<td>'
                + '<table bgcolor="#ffffff" class="content" align="center" cellpadding="0" cellspacing="0" border="0">'
                + '<tr>'
                + '<td style="background-color: rgb(0, 82, 126)" class="header">'
                + '<table width="70" align="left" border="0" cellpadding="0" cellspacing="0">  '
                + '<tr>'
                + '<td height="70" style="padding: 0 20px 20px 0;">'
                + '<img class="fix" src="images/icon.gif" width="70" height="70" border="0" alt="" />'
                + '</td>'
                + '</tr>'
                + '</table>'
                + '<table class="col425" align="left" border="0" cellpadding="0" cellspacing="0" style="width: 100%;'
                  + ' max-width: 425px;">'
                + '<tr>'
                + '<td height="70">'
                + '<table width="100%" border="0" cellspacing="0" cellpadding="0">'
                + '<tr>'
                + '<td class="h1" style="color: #fff; padding: 5px 0 0 0;">'
                + 'You have been invited!'
                + '</td>'
                + '</tr>'
                + '</table>'
                + '</td>'
                + '</tr>'
                + '</table>'
                + '</td>'
                + '</tr>'
                + '<tr>'
                + '<td class="innerpadding borderbottom">'
                + '<table width="100%" border="0" cellspacing="0" cellpadding="0">'
                + '<tr>'
                + '<td class="h2">'
                + 'Participate in a tour through ' + booking.local.city
                + '</td>'
                + '</tr>'
                + '<tr>'
                + '<td class="bodycopy">'
                + 'I just booked a tour on FindALocal.com. It will take place on ' + dateFormat(booking.date, "mmmm dd, h:MM:ss TT")
                  + ' and I want you to join! Our host ' + booking.local.displayname + ' will show us places that even regular tour'
                  + ' guides have never heard about! If you are interested, just click below.'
                + '</td>'
                + '</tr>'
                + '<tr>'
                + '<td style="padding: 20px 0 0 0;">'
                + '<table class="buttonwrapper" bgcolor="#e05443" border="0" cellspacing="0" cellpadding="0">'
                + '<tr>'
                + '<td class="button" height="45">'
                + '<a href="' + BASEURL + '/#/profile/' + booking.local._id + '">Participate!</a>'
                + '</td>'
                + '</tr>'
                + '</table>'
                + '</td>'
                + '</tr>'
                + '</table>'
                + '</td>'
                + '</tr>'
                + '<tr>'
                + '<td class="footer" style="background-color: rgb(0, 38, 58)">'
                + '<table width="100%" border="0" cellspacing="0" cellpadding="0">'
                + '<tr>'
                + '<td align="center" class="footercopy">'
                + '© 2016 FindALocal.com'
                + '</td>'
                + '</tr>'
                + '</table>'
                + '</td>'
                + '</tr>'
                + '</table>'
                + '</td>'
                + '</tr>'
                + '</table>'
                + '</body>'
                + '</html>'
    };

    console.log("mail to " + to + "is being sent now");

    transporter.sendMail(mailOptions)
};
