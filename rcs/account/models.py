from django.contrib.auth.models import AbstractBaseUser, BaseUserManager, PermissionsMixin
from django.db import models
from django.conf import settings


class UserManager(BaseUserManager):
    use_in_migrations = True

    def get_by_natural_key(self, username):
        return self.get(**{f"{self.model.USERNAME_FIELD}__iexact": username})

    def create_user(self, username, password, is_superuser):
        user = self.model(username=username)
        user.set_password(password)
        user.is_superuser = is_superuser
        user.save()

        return user

    def create_superuser(self, username, password):
        super_user = self.create_user(username=username, password=settings.DJANGO_SUPERUSER_PASSWORD, is_superuser=True)

        return super_user


class User(AbstractBaseUser, PermissionsMixin):
    username = models.TextField(unique=True)
    create_time = models.DateTimeField(auto_now_add=True, null=True)
    is_superuser = models.BooleanField(default=False)
    is_staff = models.BooleanField(default=True)

    USERNAME_FIELD = "username"
    REQUIRED_FIELDS = []

    objects = UserManager()

    class Meta:
        db_table = 'rcs_user'


class UserProfile(models.Model):
    user = models.OneToOneField(User, on_delete=models.CASCADE, unique=True)
    real_name = models.CharField(max_length=64, null=True)
    settings = models.JSONField(default=dict)
    department = models.CharField(max_length=64, null=True)

    class Meta:
        db_table = 'rcs_user_profile'
