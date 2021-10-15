from build import planner_module


r = planner_module.add(1, 2, (1,2))
print(r)

planner_module.plan('/home/zhihui/global-planner-autors2/map.png', (10, 10), (900, 200))