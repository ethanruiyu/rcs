from rest_framework.response import Response
from rest_framework.status import HTTP_200_OK, HTTP_500_INTERNAL_SERVER_ERROR


def success_response(headers=None, status=HTTP_200_OK, detail='success', data=''):
    return Response(status=status, data={
        'detail': detail,
        'data': data
    }, headers=headers)


def error_response(headers=None, status=HTTP_500_INTERNAL_SERVER_ERROR, detail='error', data=''):
    return Response(status=status, data={
        'detail': detail,
        'data': data
    }, headers=headers)