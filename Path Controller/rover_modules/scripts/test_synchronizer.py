# Unit tests for base_synchronizer
from rover_modules.synchronizer import Synchronizer

FAILURES = 0

def test_init(max_size):
    for i in xrange(max_size, 0, -1):
        print('\ntest_init: Checking proper initialization of buffer size ' + 
              str(i))
        try:
            sync = Synchronizer(i)
            print_status('test_init', 'buffer size test ' + str(i), 
                         sync.buffer['levels'] == i)
            print_status('test_init', 'trash test ' + str(i), 
                         not sync.trash)
            print_status('test_init', 'lock test ' + str(i), sync.lock)
        except SystemExit as e:
            print(e.code)
            print_status('test_init', 'sys exit test ', i < 2)

def construct_groups(size):
    groups = list()
    groups.append(['1.' + str(j + 1) for j in xrange(size)])
    groups.append([groups[0][j] if j + 1 is not size else '2.' + str(size)
                   for j in xrange(size)])
    groups.append([groups[0][j] if j + 1 is not size - 1 else '2.' + 
                   str(size - 1) for j in xrange(size)])
    groups.append([groups[2][j] if j + 1 is not size else '2.' + str(size)
                   for j in xrange(size)])

    print 'group 1: ' + str(groups[0])
    print 'group 2: ' + str(groups[1])
    print 'group 3: ' + str(groups[2])
    print 'group 4: ' + str(groups[3])
    return groups

def test_1(sync, groups, des_output, func):
    func(groups[0])
    func(groups[0])
    func(groups[0])
    buf_test_1 = sync.sync()
    print_status('test_1', 'Test single group' + ', buffer:' + str(buf_test_1),
                 buf_test_1 == des_output[0])
    buf_test_2 = sync.sync()
    print_status('test_1', 'Test single group (second sync)'+ 
                 ', buffer:' + str(buf_test_2), 
                 buf_test_2 == des_output[1])

def test_2(sync, groups, des_output, func):
    func(groups[0])
    func(groups[1])
    func(groups[2])
    func(groups[3])
    buf_test_1 = sync.sync()
    print_status('test_2', 'Test mixed group' + ', buffer:' + str(buf_test_1),
                 buf_test_1 == des_output[0])
    buf_test_2 = sync.sync()
    print_status('test_2', 'Test mixed group (second sync)' + ', buffer:' + 
                 str(buf_test_2), 
                 buf_test_2 == des_output[1])

def test_push(max_size):
    outputs = [[{'1.1':{'1.2':{'1.3':'1.4'}}},
                {'1.1':{'1.2':'1.3'}}, 
                {'1.1':'1.2'}],

               [{'1.1':{'1.2':{'1.3':'2.4', '2.3':'2.4'}}},
                {'1.1':{'1.2':'2.3', '2.2':'2.3'}},
                {'1.1': '2.2', '2.1': '2.2'}]]

    for i in xrange(max_size, 1, -1):
        print('\ntest_push: Checking function for size ' + str(i))
        groups = construct_groups(i)
        sync = Synchronizer(i)
        test_1(sync, 
               groups,
               [outputs[0][max_size - i],outputs[0][max_size - i]],
               sync.push) 
        
        sync = Synchronizer(i)
        test_2(sync,
               groups,
               [outputs[1][max_size - i], outputs[1][max_size - i]],
               sync.push)

def test_pop(max_size):
    outputs = [[{'1.1':{'1.2':{'1.3':'1.4'}}},
                {'1.1':{'1.2':'1.3'}}, 
                {'1.1':'1.2'}],

               [{'1.1':{'1.2':{'1.3':'2.4', '2.3':'2.4'}}},
                {'1.1':{'1.2':'2.3', '2.2':'2.3'}},
                {'1.1': '2.2', '2.1': '2.2'}]]

    for i in xrange(max_size, 1, -1):
        print('\ntest_pop: Checking function for size ' + str(i))
        groups = construct_groups(i)
        sync = Synchronizer(i)
        test_1(sync,
               groups,
               [outputs[0][max_size - i], {}],
               sync.pop)
        
        sync = Synchronizer(i)
        test_2(sync,
               groups,
               [outputs[1][max_size - i], {}],
               sync.pop)

def test_check_size(max_size):
    keys = [[['1.1'],
             ['1.1','1.2'],
             ['1.1','1.2', '1.3'],
             ['1.1','1.2', '1.3', '1.4']],
            [['1.1'],
             ['1.1','1.2'],
             ['1.1','1.2', '1.3']],
            [['1.1'],
             ['1.1','1.2']],
            [['1.1']]]
    outputs = [[[1,1,1,0],
                [1,1,0],
                [1,0]]]
    for i in xrange(max_size, 1, -1):
        print('\ntest_check_size: Checking function for size ' + str(i))
        groups = construct_groups(i)
        sync = Synchronizer(i)
        sync.push(groups[0])
        sync.push(groups[0])
        sync.push(groups[0]) 
        data = sync.sync()
        for idx, key in enumerate(keys[max_size - i]):
            buf_size = sync.check_buffer_size(key)
            des_output = outputs[0][max_size - i][idx]
            print_status('test_check_size', 'Test single group check size key: '
                         + str(key) + ', actual output: ' + str(buf_size) + 
                         ', desired output: ' + str(des_output) + ' buffer: ' + 
                         str(data), buf_size == des_output)
        
def print_status(func, message, success):
    global FAILURES
    print(func + ': Success, ' + message if success else func + ': Failed, ' +
          message)
    if not success: FAILURES += 1

if __name__ == '__main__':
    max_size = 4
    test_init(max_size)
    test_push(max_size)
    test_pop(max_size)
    test_check_size(max_size)
    print('Failed ' + str(FAILURES) + ' tests' if FAILURES
          else 'Passed all tests!')
