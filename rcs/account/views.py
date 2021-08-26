from rest_framework.viewsets import ModelViewSet
from .serializers import *
from rest_framework_simplejwt.authentication import JWTAuthentication
from rest_framework.permissions import IsAuthenticated
from rest_framework_simplejwt.views import TokenObtainPairView


class UserProfileViewSet(ModelViewSet):
    serializer_class = UserProfileSerializer
    queryset = UserProfile.objects.all()
    authentication_classes = (JWTAuthentication,)
    permission_classes = [IsAuthenticated]


class RCSTokenObtainPairView(TokenObtainPairView):
    serializer_class = RCSTokenObtainPairSerializer
