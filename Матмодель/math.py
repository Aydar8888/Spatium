import matplotlib.pyplot as plt
from math import atan, sqrt, exp, sin, cos, pi

#Вовзращает 1 если x > 0, -1 если x < 0, инчаче 0
def sign(x):
    if x > 0:
        return 1
    if x < 0:
        return -1
    return 0

'''Обновляет значения глобальных переменных, описывающих состояние 
ракеты в следующий момент времени, используя текущие данные и вычисления на основе 
физических формул. Она имитирует шаг во времени для симуляции движения ракеты.'''
def next_step():
    global x, y, h, v, a, t_now, mass_now, v_x, v_y
    x_tmp = next_x() #координата x в следующий момент времени, вычисляется с помощью функции next_x()
    y_tmp = next_y() #координата y в следующий момент времени, вычисляется с помощью функции next_y()
    v_x_tmp = next_velocity_x() #проекция скорости на ось x в следующий момент времени, вычисляется с помощью функции next_velocity_x()
    v_y_tmp = next_velocity_y() #проекция скорости на ось y в следующий момент времени, вычисляется с помощью функции next_velocity_y()
    v_tmp = velocity() #общая скорость ракеты в следующий момент времени, вычисляется с помощью функции velocity()
    h_tmp = altitude() #высота ракеты над поверхностью, вычисляется с помощью функции altitude()
    t_now += delta_t #прибавляет шаг времени (Δt) к текущему времени. Это обеспечивает переход симуляции к следующему моменту.
    a = acceleration(t_now) #рассчитывает ускорение ракеты с помощью функции acceleration() на основе текущего времени.
    v = v_tmp #обновляет общую скорость ракеты.
    x = x_tmp # обновляет координату x
    y = y_tmp #обновляет координату y
    v_x = v_x_tmp #обновляет проекцию скорости на ось x
    v_y = v_y_tmp #обновляет проекцию скорости на ось y
    h = h_tmp #обновляет высоту ракеты
    mass_now = mass(t_now) #обновляет текущую массу ракеты с помощью функции mass(), которая учитывает расход топлива


#Функция моделирует время, возращая лист
def create_time_list(N):
    return [i * delta_t for i in range(N)]

#Вычисляет новые координаты x
def next_x():
    return x + v_x * delta_t

#Вычисляет новые координаты y
def next_y():
    return y + v_y * delta_t


# коэф для проекции на ось Oy
def y_coef_fi():
    ang_teta = abs(teta())
    ang_gamma = abs(gamma())
    if x * y >= 0 and ang_teta + pi / 2 - ang_gamma <= pi / 2:
        return -abs(sin(ang_teta + ang_gamma))
    elif x * y >= 0 and ang_teta + pi / 2 - ang_gamma > pi / 2:
        return -abs(sin(abs(ang_teta + ang_gamma)))
    elif x * y < 0 and (ang_teta - ang_gamma) <= 0:
        return sin(abs(ang_gamma - ang_teta))
    elif x * y < 0 and (ang_teta - ang_gamma) > 0:
        return -sin(abs(ang_gamma - ang_teta))


# коэф для проекции на ось Ох
def x_coef_fi():
    ang_teta = abs(teta())
    ang_gamma = abs(gamma())
    if x * y >= 0 and ang_teta + ang_gamma <= pi / 2:
        return -abs(cos(ang_teta + ang_gamma))
    elif x * y >= 0 and ang_teta + ang_gamma > pi / 2:
        return abs(cos(ang_teta + ang_gamma))
    elif x * y < 0 and ang_teta + ang_gamma <= pi / 2:
        return -abs(cos(abs(ang_gamma - ang_teta)))
    elif x * y < 0 and ang_teta + ang_gamma > pi / 2:
        return -abs(cos(abs(ang_gamma - ang_teta)))

#Вычисляет текущую массу ракеты на основе времени, расхода топлива и масс ступеней.
def mass(time_now):
    global real_mass

    if time_now <= t1:
        real_mass = initial_mass - time_now * fuel_consumption1
    elif time_now <= t2:
        real_mass = initial_mass - m_stage1
    elif time_now <= t3:
        real_mass = initial_mass - (time_now - t2) * fuel_consumption2 - m_stage1
    return real_mass

#Расчитывает высоту над поверхностьтю
def altitude():
    return sqrt(x ** 2 + y ** 2) - r


# сила тяги
def traction_force(time_now):
    global specific_impulse1, specific_impulse2, specific_impulse3, specific_impulse4

    if time_now <= t1:
        return specific_impulse1[0] 
    elif time_now <= t2:
        return 0
    elif time_now <= t3:
        return specific_impulse2[1]
    return 0


# 90 - крена
def gamma():
    if x == 0:
        return pi / 2
    return atan(y / x)


# тангажа + 90 - крена
def teta():
    pos0, pos1 = (18500, 0), (58500, pi / 2)
    p, q = pos0
    s, r = pos1
    a = s - p
    b = r - q
    if h < pos0[0]:
        return pos0[1]
    if h > pos1[0]:
        return pos1[1]
    return sqrt(b ** 2 - (b / a) ** 2 * (h - s) ** 2) + q


# Ускорение свободного падения
def acceleration_of_gravity():
    center_distance = (r + h)
    return (G * M) / (center_distance ** 2)


# сида тяжести
def gravity(time_now):
    chill_coef = 0.83
    if h > 45000:
        return mass_now * acceleration_of_gravity() * chill_coef
    return mass_now * acceleration_of_gravity()


# плотность воздуха
def air_density() -> float:
    if t_now < 40:
        return p0 * exp(-acceleration_of_gravity() * m * h / (R * (t0 - 6.5 * (h / 1000))))
    return p0 * exp(-acceleration_of_gravity() * m * h / (R * (t0 - 240)))


# сила сопротивления воздуха, Fсопр
def air_resistance_force():
    return 0.5 * Cd * S * v ** 2 * air_density()


def acceleration_x(time_now):
    global a_x
    b = traction_force(time_now) * x_coef_fi()
    c = air_resistance_force() * x_coef_fi()
    d = sign(x) * gravity(time_now) * cos(abs(gamma()))
    e = mass_now
    a_x = (b - c - d) / e
    return a_x


def acceleration_y(time_now):
    global a_y
    b = traction_force(time_now) * y_coef_fi()
    c = air_resistance_force() * y_coef_fi()
    d = sign(y) * gravity(time_now) * sin(abs(gamma()))
    e = mass_now
    a_y = (b - c - d) / e
    return a_y



def acceleration(time_now):
    return sqrt(acceleration_x(time_now) ** 2 + acceleration_y(time_now) ** 2)


def next_velocity_x():
    return v_x + a_x * delta_t


def next_velocity_y():
    return v_y + a_y * delta_t


# скорость в следующий момент времени, v
def velocity() -> float:
    return sqrt(v_x ** 2 + v_y ** 2)


# графики
def plots():
    velocity_list = []
    x_list = []
    y_list = []
    a_list = []
    h_list = []
    mass_list = []
    teta_list = []
    gamma_list = []
    a_x_list = []
    a_y_list = []
    x_okr = [i for i in range(-600_000, 600_001)]
    y1 = [sqrt(r ** 2 - x ** 2) for x in x_okr]
    y2 = [-sqrt(r ** 2 - x ** 2) for x in x_okr]

    for i in time_list:
        velocity_list.append(v)
        mass_list.append(mass_now)
        x_list.append(x)
        y_list.append(y)
        a_list.append(a)
        h_list.append(h)
        a_x_list.append(a_x)
        a_y_list.append(a_y)
        teta_list.append(teta() * 180 / pi)
        gamma_list.append(gamma() * 180 / pi)
        next_step()

    tmp, plot = plt.subplots()
    plot.set_title('Изменение скорости ракеты')
    plot.set_xlabel('Время в секундах')
    plot.set_ylabel('Скорость в м/с')
    plot.plot(time_list, velocity_list)
    plot.plot(ksp_time_list, ksp_velocity_list)
    plot.grid()

    tmp, plot1 = plt.subplots()
    plot1.set_title('Изменение положения ракеты от-но центра Кербина')
    plot1.set_xlabel('х кооридната в м')
    plot1.set_ylabel('y координата в м')
    plot1.plot(x_list, y_list)
    plot1.plot(ksp_x_coor_list, ksp_y_coor_list)
    plot1.plot(x_okr, y1, color='blue')
    plot1.plot(x_okr, y2, color='blue')
    plot1.grid()

    tmp, plot2 = plt.subplots()
    plot2.set_title('Изменение высоты')
    plot2.set_xlabel('Время в секундах')
    plot2.set_ylabel('Высота в метрах')
    plot2.plot(time_list, h_list)
    plot2.plot(ksp_time_list, ksp_h_list)
    plot2.grid()

    tmp, plot4 = plt.subplots()
    plot4.set_title('Изменение массы ракеты')
    plot4.set_xlabel('Время в секундах')
    plot4.set_ylabel('Масса в килограммах')
    plot4.plot(time_list, mass_list, label='Рассчетные данные')
    plot4.plot(ksp_time_list, ksp_mass_list, label='ksp')
    plot4.legend('Расчетные данные', 'ksp')
    plot4.grid(True)


def ksp_data():
    global ksp_mass_list, ksp_time_list, ksp_x_coor_list, ksp_y_coor_list, ksp_h_list, ksp_velocity_list

    with open('/home/aidar/Документы/GitHub/VARKT/Autopilot/Logs/stagetime_data.txt', 'r') as file:
        lines = file.readlines()
        for line in lines:
            lst = line.split('; ')
            ksp_mass_list.append(float(lst[3]))
            ksp_time_list.append(float(lst[0]))
            ksp_x_coor_list.append(float(lst[4]))
            ksp_y_coor_list.append(float(lst[6]))
            ksp_h_list.append(float(lst[1]))
            ksp_velocity_list.append(float(lst[2]))



# константы атмосферы Кербина
p0 = 1.2255  # плотность воздуха на уровне моря, кг/м^3
t0 = 250.15  # стандартная температурана экваторе в точке старта, Kельвины
m = 0.029  # молярная масса сухого воздуха, кг/моль
R = 8.31  # универсальная газовая постоянная, Дж/(моль * Кельвины)

# физические константы Кербина
G = 6.6743015e-11  # гравитационная постоянная, (м^3) / (c^2 * кг^2)
r = 600_000  # радиус Кербина, м
M = 5.2915158e22  # масса Кербина, кг

# время
N = 100_000  # количество пересчетов
t1 = 80.85  # время работы первой ступени
t2 = 394.392  # время работы второй ступени
t3 = 425  # время работы третий ступени
t = t3 + 1  # общее время полета в секундах
delta_t = t / N
time_list = create_time_list(N)


initial_mass = 482000  # масса ракеты-носителя и полезного груза в начальный момент времени
m_stage1 = initial_mass - 61200  # масса первой ступени, кг 61957
Cd = 0.5  # обтекание сферы
d = 1.5  # диаметр корпуса ракеты
S = d ** 2 * pi  # 0.008 * m_without_fuel 

# параметры двигателей
fuel_consumption2 = (61000 - 45000) / (t3 - t2)  # расход массы второй ступени
fuel_consumption1 = (482000 - 124000) / t1  # расход массы первой ступени
specific_impulse1 = [1_515_217 * 6, 1_700_000 * 6]
specific_impulse2 = [1_379_000, 1_500_000]

real_mass = initial_mass

# списки данных из ksp
ksp_time_list = []
ksp_mass_list = []  # список масс космического аппарата
ksp_velocity_list = []  # список скоростей космического аппарата
ksp_x_coor_list = []  # список х координат космического аппарата
ksp_y_coor_list = []  # список у координат космического аппарата
ksp_h_list = []  # список высот космического аппарата
ksp_data()

# исходные данные о полете
x0 = -28993.843443421472  # начальная х координата
y0 = 599382.1908592838  # начальная у координата
h0 = 85  # начальная высота
v0 = 0  # начальная скорость

# глобальные переменные: x, y, h, v, a
x = x0
y = y0
h = h0
v = v0
v_x = v * x_coef_fi()
v_y = v * y_coef_fi()
t_now = 0
mass_now = initial_mass
a = acceleration(0)
a_x = acceleration_x(0)
a_y = acceleration_y(0)

plots()
plt.show()
