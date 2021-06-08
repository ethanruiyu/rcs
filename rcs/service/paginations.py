from rest_framework.pagination import PageNumberPagination
from rest_framework.response import Response
from collections import OrderedDict


class MapPagination(PageNumberPagination):
    page_size = 5
    page_size_query_param = 'pageSize'
    page_query_param = 'pageNo'
    max_page_size = 100

    def get_paginated_response(self, data):
        return Response(OrderedDict([
            ('next', self.get_next_link()),
            ('pageNo', self.page.number),
            ('previous', self.get_previous_link()),
            ('count', self.page.paginator.count),
            ('results', data)
        ]))
