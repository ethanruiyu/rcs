from rest_framework.response import Response
from rest_framework.status import HTTP_200_OK


def success_response(headers=None, status=HTTP_200_OK, detail='success', data='', code=0):
    return Response(status=status, data={
        'detail': detail,
        'code': code,
        'data': data
    }, headers=headers)


def error_response(headers=None, status=HTTP_200_OK, detail='error', data='', code=-1):
    return Response(status=status, data={
        'detail': detail,
        'code': code,
        'data': data
    }, headers=headers)