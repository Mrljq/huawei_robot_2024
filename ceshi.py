import timeit

# 定义一个简单的函数，该函数包含一个 for 循环
def test_for_loop():
    result = 0
    for i in range(20000):
        result += i
    return result

# 使用 timeit 测试该函数的运行时间
time_taken = timeit.timeit(test_for_loop, number=1000)  # 运行1000次以获取更稳定的平均时间

print(f"Average time taken for the loop: {time_taken:.6f} seconds")
