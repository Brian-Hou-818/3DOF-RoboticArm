from sympy import symbols, Eq, solve
import math

def get_values(Fdeg, Ideg, Ft, It, Fvel, Ivel):
    a0, a1, a2, a3 = symbols('a0, a1, a2, a3')
    eq1 = Eq((a0 + a1 * Ft + a2 * Ft ** 2 + a3 * Ft ** 3), Fdeg)
    eq2 = Eq((a0 + a1 * It + a2 * It ** 2 + a3 * It ** 3), Ideg)
    eq3 = Eq((a1 + 2 * a2 * It + 3 * a3 * It ** 2), Ivel)
    eq4 = Eq((a1 + 2 * a2 * Ft + 3 * a3 * Ft ** 2), Fvel)
    d = solve((eq1, eq2, eq3, eq4), (a0, a1, a2, a3))
    list = [*d.values()]
    return list

move_X = int(input("X: "))
move_Y = int(input("Y: "))
move_Z = int(input("Z: "))

if move_X == 0:
    move_X = 0.001
if move_Y == 0:
    move_Y = 0.001
if move_Z == 0:
    move_Z = 0.001

Fdeg = round(math.degrees(math.atan(move_Z / math.sqrt(move_X ** 2 + move_Y ** 2))), 2)#calculate Z angle
Ideg = 0
Fvel = 0
Ivel = 0
Ft = int(input("Time: ")) #Time
It = 0 #initial time
values = get_values(Fdeg, Ideg, Ft, It, Fvel, Ivel)
print(values)

angle = round(values[0] + values[1] * Ft + values[2] * Ft ** 2 + values[3] * Ft ** 3, 2)
print("angle =", angle)

Ideg = Fdeg