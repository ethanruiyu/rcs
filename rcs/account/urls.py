from rest_framework.routers import DefaultRouter
from django.urls import path
from rcs.account.views import *

account_router = DefaultRouter(trailing_slash=False)
account_router.register(basename='', prefix='profile', viewset=UserProfileViewSet)

urlpatterns = [
]

urlpatterns += account_router.urls
