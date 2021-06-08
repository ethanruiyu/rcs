#!/bin/bash

rm -rf rcs/core/migrations
rm -rf rcs/common/migrations
rm -rf rcs/account/migrations
rm -rf rcs/adapter/migrations

python manage.py makemigrations account core
python manage.py migrate
python manage.py loaddata rcs/core/fixtures/core.json
python manage.py createsuperuser --noinput --username=rcs

