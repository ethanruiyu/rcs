from rest_framework.pagination import PageNumberPagination
from collections import OrderedDict
from ..common.utils.response import success_response


class MissionPagination(PageNumberPagination):
    # page_size = 10
    page_size_query_param = 'pageSize'
    page_query_param = 'pageNo'
    # max_page_size = 100

    def get_paginated_response(self, data):
        return success_response(data=OrderedDict([
            ('next', self.get_next_link()),
            ('pageNo', self.page.number),
            ('previous', self.get_previous_link()),
            ('count', self.page.paginator.count),
            ('results', data)
        ]))
