#!/usr/bin/env python
from bottle import post, run, template, request, response

@post('/log')
def log():
	response.status = 200
	return

run(host='localhost', port=5000)
