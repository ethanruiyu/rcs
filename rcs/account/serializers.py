from rest_framework import serializers
from rcs.account.models import *
from rest_framework_simplejwt.serializers import TokenObtainPairSerializer


class UserProfileSerializer(serializers.ModelSerializer):

    class Meta:
        model = UserProfile
        fields = '__all__'


class RCSTokenObtainPairSerializer(TokenObtainPairSerializer):
    # def create(self, validated_data):
    #     super(RCSTokenObtainPairSerializer, self).create(validated_data)
    #
    # def update(self, instance, validated_data):
    #     super(RCSTokenObtainPairSerializer, self).update(validated_data)
    #
    def validate(self, attrs):
        data = super().validate(attrs)

        refresh = self.get_token(self.user)

        data['refresh'] = str(refresh)
        data['access'] = str(refresh.access_token)
        data['user'] = {
            'id': self.user.id
        }
        return data

    def get_token(cls, user):
        token = super().get_token(user)
        token['name'] = user.username
        print(token)
        return token