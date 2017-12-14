#!/usr/bin/env python
from bottle import post, run, template, BaseResponse

@post('/log')
def log():
	return BaseResponse(body="", status=200, headers=None)

run(host='localhost', port=5000)
