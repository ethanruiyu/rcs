#!/bin/bash
export DJANGO_SUPERUSER_PASSWORD=123456

python manage.py makemigrations
python manage.py migrate
#python manage.py loaddata fixtures
python manage.py createsuperuser --noinput --username=rcs --email=rcs@rcs.com

