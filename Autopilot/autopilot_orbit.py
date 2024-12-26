#!/home/aidar/Документы/GitHub/VARKT/venv/bin/python
import krpc
import time
import math
import threading


# # Cбор данных
# def telem():
#     while True:
#         t = flight_time() # время
#         h = vessel.flight().mean_altitude # высота
#         v = vessel.flight().true_air_speed # скорость
#         m = vessel.mass # масса
#         coord1 = vessel.position(conn.space_center.bodies['Kerbin'].non_rotating_reference_frame) # координаты отн. земли
#         coord2 = vessel.position(conn.space_center.bodies['Sun'].non_rotating_reference_frame) # координаты отн. солнца
#         file = open("Autopilot/Logs/stagetime_data.txt", 'a')
#         file.write("; ".join(list(map(str, [t, h, v, m] + [coord1[0]] + [coord1[1]] + [coord1[2]] + [coord2[0]] + [coord2[1]] + [coord2[2]]))))
#         file.write("\n")
#         file.close()
#         time.sleep(1)

# def flight_time():
#     return str(conn.space_center.ut - ut_start)

data_time = []
conn = krpc.connect()  # подключаемся к серверу 
vessel = conn.space_center.active_vessel  # активный корабль
control = vessel.control  # контролировать корабль
ap = vessel.auto_pilot  # работать с автопилотом

ut_start = conn.space_center.ut
ut_now = conn.space_center.ut
# готовимся к запуску
ap.target_pitch_and_heading(90, 90)
ap.engage()

# переменные потоки, при вызове которых мы получаем данные из KSP
ut = conn.add_stream(getattr, conn.space_center, 'ut')  # текущее время в KSP
stage_4_resources = vessel.resources_in_decouple_stage(stage=5, cumulative=False)  # пятая ступень (та, где отделяются ускорители)
srb_fuel = conn.add_stream(stage_4_resources.amount, 'SolidFuel')  # количество топлива во всех ускорителях в сумме
altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')  # высота над уровнем моря в метрах
apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')  # высота апоцентра в метрах, если считать от уровня моря
periapsis = conn.add_stream(getattr, vessel.orbit, 'periapsis_altitude')  # высота перицентра в метрах, если счтиать от уровня моря
pitch = conn.add_stream(getattr, vessel.flight(), 'pitch')  # рысканье ракеты



# запускаем ракету
print("3...")
time.sleep(0.5)
print("2...")
time.sleep(0.5)
print("1...")
time.sleep(0.5)

print("Start!")
control.activate_next_stage()
start_time = time.time()
# threading.Thread(target=telem).start()


# параметры, с которыми ракета будет наклоняться в виде (высота апоцентра, угол рысканья)
# первая точка, это какой угол должен быть в начале его изменения и на какой высоте апоцентра начать его менять
# вторая точка, это значение угла в конце его изменения и высота апоцентра, на котором угол должен быть достигнут
pos0, pos1 = (18500, 0), (58500, math.pi / 2)
def set_elliptic(pos0, pos1):
    p, q = pos0
    s, r = pos1
    a = s - p
    b = r - q
    
    def func(x):
        if x < pos0[0] or x > pos1[0]:
            return pos0[1] if x < pos0[0] else pos1[1]
        return math.sqrt(b ** 2 - (b / a) ** 2 * (x - s) ** 2) + q

    return func


# сама функция угла
angle = set_elliptic(pos0, pos1)



"""
ВЗЛЁТ РАКЕТЫ (доорбитальный этап)
Задачи:
    1. запустить ракету
    2. плавно лечь в горизонт
    3. отбросить ускорители
"""
# ждём, пока апоцентр достигнет нужной высоты
while altitude() < pos0[0]:
   time.sleep(0.5)  

# наклоняем ракету до тех пор, пока топливо в тту не закончится
while srb_fuel() >= 0.1:
    # print(apoapsis(), angle(apoapsis()))
    ap.target_pitch = math.degrees(math.pi / 2 - angle(altitude()))
    # print(srb_fuel(), math.degrees(math.pi / 2 - angle(altitude())))
    time.sleep(0.2)
ap.target_pitch = 0

# отсоединаяем их
control.activate_next_stage()
print("Ускорители отброшены")


"""
ВЫХОД НА КРУГОВУЮ ОРБИТУ (орбитальный этап)
Задачи:
    1. Открыть солнечные панели
    2. Набрать необходимую дельта скорость
"""

# Функция, которая считает гомановский переход
def test3(mu, r1, r2):
    # принимает стандартный гравитационный параметр mu
    # радиус круговой орбиты r1, радиус круговой орбиты r2
    # r1 < r2
    a = (r1 + r2) / 2
    dv1 = math.sqrt(mu / r1) * (math.sqrt(r2 / a) - 1)
    dv2 = math.sqrt(mu / r2) * (1 - math.sqrt(r1 / a))
    # dv1 - На такое значение нужно увеличить скорость, будучи на круговой орбите r1
    # dv2 - на Такое значение нужно увеличить скорость, будучи на переходной траектории гомана (высота апоцентра это r2)
    return dv1, dv2

time.sleep(3)

# print("Добавляем ноду манёвра")
mu = vessel.orbit.body.gravitational_parameter
delta_v = test3(mu, vessel.orbit.periapsis, vessel.orbit.apoapsis)[1]
node = control.add_node(ut() + vessel.orbit.time_to_apoapsis, prograde=delta_v)
time.sleep(1)

# print("Считаем время работы двигателя")
F = vessel.available_thrust
Isp = vessel.specific_impulse * 9.82
m0 = vessel.mass
m1 = m0 / math.exp(delta_v / Isp)
flow_rate = F / Isp
burn_time = (m0 - m1) / flow_rate

print("Выключаем автопилот")
ap.disengage()
# print("Включаем САС")
control.sas = True
time.sleep(1)
# print("Ставим САС на манёвр")
control.sas_mode = conn.space_center.SASMode.maneuver

print("Ждём момента до ускорения")
burn_ut = ut() + vessel.orbit.time_to_apoapsis - (burn_time/2)
lead_time = 5
time_to_apoapsis = conn.add_stream(getattr, vessel.orbit, "time_to_apoapsis")
while time_to_apoapsis() - (burn_time / 2) > 0:
	time.sleep(0.05)
    
print("Запускаем двигатели")
control.throttle = 1.0
time_when_end = ut() + vessel.orbit.time_to_apoapsis + burn_time - 16
while ut() < time_when_end:
    time.sleep(0.05)


print("Ракета успешно выведена на орбиту 250 км")
control.activate_next_stage()
control.throttle = 0
data_time.append(time.time() - start_time)
control.remove_nodes()

