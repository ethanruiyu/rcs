#!/bin/bash

rm -rf rcs/core/migrations
rm -rf rcs/common/migrations
rm -rf rcs/account/migrations
rm -rf rcs/adapter/migrations
rm -rf rcs/map/migrations

python manage.py makemigrations account map common vehicle mission
python manage.py migrate
python manage.py loaddata fixtures.json
python manage.py createsuperuser --noinput --username=rcs

