"""rcs URL Configuration

The `urlpatterns` list routes URLs to views. For more information please see:
    https://docs.djangoproject.com/en/3.2/topics/http/urls/
Examples:
Function views
    1. Add an import:  from my_app import views
    2. Add a URL to urlpatterns:  path('', views.home, name='home')
Class-based views
    1. Add an import:  from other_app.views import Home
    2. Add a URL to urlpatterns:  path('', Home.as_view(), name='home')
Including another URLconf
    1. Import the include() function: from django.urls import include, path
    2. Add a URL to urlpatterns:  path('blog/', include('blog.urls'))
"""
from django.conf.urls import url
from django.contrib import admin
from django.urls import path, include
from django.views.static import serve
from drf_spectacular.views import SpectacularAPIView, SpectacularSwaggerView
from rest_framework_simplejwt.views import TokenRefreshView
from django.conf import settings
from .communication.master import Master
from rcs.account.views import UserProfileViewSet, RCSTokenObtainPairView
# from rcs.adapter.adapter import MasterMqttAdapter
# from rcs.core.models import MapModel
from rcs.simulator.views import Simulator

urlpatterns = [
    path('admin/', admin.site.urls),
    # path('api/', include('rcs.service.urls')),
    path('api/account/', include('rcs.account.urls')),
    path('api/', include('rcs.map.urls')),
    path('api/', include('rcs.vehicle.urls')),
    path('api/', include('rcs.mission.urls')),
    path('api/schema/', SpectacularAPIView.as_view(), name='schema'),
    path('api/schema/doc/',
         SpectacularSwaggerView.as_view(url_name='schema'), name='swagger-ui'),

    path('api/token/', RCSTokenObtainPairView.as_view(), name='token_obtain_pair'),
    path('api/token/refresh/', TokenRefreshView.as_view(), name='token_refresh'),

    url(r'media/(?P<path>.*)', serve,
        {"document_root": settings.MEDIA_ROOT, 'show_indexes': True}),
]

# sim = Simulator('Robot-1')
# sim.enable()



master = Master()
master.enable()

# if MapModel.objects.filter(active=True).exists():
#     settings.ACTIVE_MAP_CONFIG = MapModel.objects.filter(active=True).first().config
