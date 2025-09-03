import random

def draw_lottery(area, options, numbers):
    # 抽取一个选项
    selected_option = random.choice(options)
    print(area + "区固定图像板：", selected_option)
    # 抽取四个数字
    selected_numbers = random.sample(numbers, 4)
    print(area + "区随机图像板：", selected_numbers)
    print()

print("---------------------- 各区结果 -------------------------------") 
items = ["水稻", "小麦", "玉米", "黄瓜"]
letters = ["B", "D", "C", "E"]
random.shuffle(items)
results = [letter + item for letter, item in zip(letters, items)]
for result in results:
    print(result)
print()

print("---------------------- B区抽签结果 -------------------------------")
options_b = ['B左', 'B左上', 'B右', 'B右上']
numbers_b = [13, 23, 33, 43, 14, 24, 34, 44, 35, 45]
draw_lottery("B", options_b, numbers_b)

print("---------------------- D区抽签结果 -------------------------------")
options_d = ["D左", "D左上", "D右", "D右上"]
numbers_d = [53, 73, 54, 64, 74, 55, 75]
draw_lottery("D", options_d, numbers_d)

print("---------------------- C区抽签结果 -------------------------------")
options_c = ["C左", "C左下", "C右", "C右下"]
numbers_c = [38, 48, 19, 29, 39, 49, 110, 210, 310, 410]
draw_lottery("C", options_c, numbers_c)

print("---------------------- E区抽签结果 -------------------------------")
options_e = ["E左", "E下"]
numbers_e = [58, 78, 59, 69, 79, 510, 710]
draw_lottery("E", options_e, numbers_e)

print("---------------------- F区抽签结果 -------------------------------")
options_f = ["F左", "F左上", "F右", "F右上"]
a_area = [93, 103, 113, 94, 104, 114, 95, 105, 115]
b_area = ["b左", "b右"]
c_area = ["c左", "c右"]
# 从a区抽取两个数字
a_result = random.sample(a_area, 2)
# 从b区抽取一个元素
b_result = random.choice(b_area)
# 从c区抽取一个元素
c_result = random.choice(c_area)
selected_option = random.choice(options_f)
print("F区固定图像板：", selected_option)
print("a区：", a_result)
print("b区：", b_result)
print("c区：", c_result)
print()

print("---------------------- 障碍板 -------------------------------")
numbers_obstacle = [16, 26, 36, 46, 56, 66, 76, 17, 27, 37, 47, 57, 67, 77, 810, 811, 812, 910, 911, 912]
selected_numbers_obstacle = random.sample(numbers_obstacle, 3)
print("障碍板：", selected_numbers_obstacle)