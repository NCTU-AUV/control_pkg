
def get_key_by_val(target_dict, val):
    for key, value in target_dict.items():
        if val == value:
            return key

    return "key doesn't exist"
