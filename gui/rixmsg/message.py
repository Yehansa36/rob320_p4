import struct
import abc


class Message(abc.ABC):
    @abc.abstractmethod
    def serialize(self, buffer: bytearray) -> None:
        pass

    @abc.abstractmethod
    def deserialize(self, buffer: bytearray, context: dict) -> None:
        pass

    @abc.abstractmethod
    def size(self) -> int:
        pass

    @abc.abstractmethod
    def hash(self) -> int:
        pass

    @staticmethod
    def _serialize_int8(
        value: int | list[int],
        buffer: bytearray,
        size: int | None = None,
        fixed: bool = False,
    ) -> None:

        if size is not None:
            if not fixed:
                Message._serialize_uint32(size, buffer)
            buffer.extend(struct.pack(f"{size}b", *value))
        else:
            buffer.extend(struct.pack("b", value))

    @staticmethod
    def _serialize_uint8(
        value: int | list[int],
        buffer: bytearray,
        size: int | None = None,
        fixed: bool = False,
    ) -> None:

        if size is not None:
            if not fixed:
                Message._serialize_uint32(size, buffer)
            buffer.extend(struct.pack(f"{size}B", *value))
        else:
            buffer.extend(struct.pack("B", value))

    @staticmethod
    def _serialize_char(
        value: int | list[int],
        buffer: bytearray,
        size: int | None = None,
        fixed: bool = False,
    ) -> None:
        # if size is not None:
        #     if not fixed:
        #         Message._serialize_uint32(size, buffer)
        #     buffer.extend(struct.pack(f"{size}c", *value))
        # else:
        #     buffer.extend(struct.pack("c", value))
        # Serialize char as int8 (problem with struct.pack with char array represented as a number array)
        return Message._serialize_int8(value, buffer, size, fixed)

    @staticmethod
    def _serialize_bool(
        value: int | list[int],
        buffer: bytearray,
        size: int | None = None,
        fixed: bool = False,
    ) -> None:

        if size is not None:
            if not fixed:
                Message._serialize_uint32(size, buffer)
            buffer.extend(struct.pack(f"{size}?", *value))
        else:
            buffer.extend(struct.pack("?", value))

    @staticmethod
    def _serialize_int16(
        value: int | list[int],
        buffer: bytearray,
        size: int | None = None,
        fixed: bool = False,
    ) -> None:

        if size is not None:
            if not fixed:
                Message._serialize_uint32(size, buffer)
            buffer.extend(struct.pack(f"{size}h", *value))
        else:
            buffer.extend(struct.pack("h", value))

    @staticmethod
    def _serialize_uint16(
        value: int | list[int],
        buffer: bytearray,
        size: int | None = None,
        fixed: bool = False,
    ) -> None:

        if size is not None:
            if not fixed:
                Message._serialize_uint32(size, buffer)
            buffer.extend(struct.pack(f"{size}H", *value))
        else:
            buffer.extend(struct.pack("H", value))

    @staticmethod
    def _serialize_int32(
        value: int | list[int],
        buffer: bytearray,
        size: int | None = None,
        fixed: bool = False,
    ) -> None:

        if size is not None:
            if not fixed:
                Message._serialize_uint32(size, buffer)
            buffer.extend(struct.pack(f"{size}i", *value))
        else:
            buffer.extend(struct.pack("i", value))

    @staticmethod
    def _serialize_uint32(
        value: int | list[int],
        buffer: bytearray,
        size: int | None = None,
        fixed: bool = False,
    ) -> None:

        if size is not None:
            if not fixed:
                Message._serialize_uint32(size, buffer)
            buffer.extend(struct.pack(f"{size}I", *value))
        else:
            buffer.extend(struct.pack("I", value))

    @staticmethod
    def _serialize_int64(
        value: int | list[int],
        buffer: bytearray,
        size: int | None = None,
        fixed: bool = False,
    ) -> None:

        if size is not None:
            if not fixed:
                Message._serialize_uint32(size, buffer)
            buffer.extend(struct.pack(f"{size}q", *value))
        else:
            buffer.extend(struct.pack("q", value))

    @staticmethod
    def _serialize_uint64(
        value: int | list[int],
        buffer: bytearray,
        size: int | None = None,
        fixed: bool = False,
    ) -> None:

        if size is not None:
            if not fixed:
                Message._serialize_uint32(size, buffer)
            buffer.extend(struct.pack(f"{size}Q", *value))
        else:
            buffer.extend(struct.pack("Q", value))

    @staticmethod
    def _serialize_float(
        value: float | list[float],
        buffer: bytearray,
        size: int | None = None,
        fixed: bool = False,
    ) -> None:

        if size is not None:
            if not fixed:
                Message._serialize_uint32(size, buffer)
            buffer.extend(struct.pack(f"{size}f", *value))
        else:
            buffer.extend(struct.pack("f", value))

    @staticmethod
    def _serialize_double(
        value: float | list[float],
        buffer: bytearray,
        size: int | None = None,
        fixed: bool = False,
    ) -> None:

        if size is not None:
            if not fixed:
                Message._serialize_uint32(size, buffer)
            buffer.extend(struct.pack(f"{size}d", *value))
        else:
            buffer.extend(struct.pack("d", value))

    @staticmethod
    def _serialize_string(
        value: str | list[str],
        buffer: bytearray,
        size: int | None = None,
        fixed: bool = False,
    ) -> None:
        if size is not None:
            if not fixed:
                # print("vec size:" + str(size))
                Message._serialize_uint32(size, buffer)
            for item in value:
                Message._serialize_string(item, buffer)
        else:
            size = len(value)
            # print("string size:" + str(size))
            Message._serialize_uint32(size, buffer)
            buffer.extend(struct.pack(f"{size}s", value.encode()))

    @staticmethod
    def _serialize_message(
        value: object | list[object],
        buffer: bytearray,
        size: int | None = None,
        fixed: bool = False,
    ) -> None:
        if size is not None:
            if not fixed:
                Message._serialize_uint32(size, buffer)
            for item in value:
                item.serialize(buffer)
        else:
            value.serialize(buffer)

    @staticmethod
    def _serialize_map(
        value: dict, buffer: bytearray, key_func: callable, value_func: callable
    ) -> None:
        key_func(list(value.keys()), buffer, len(value), False)
        value_func(list(value.values()), buffer, len(value), False)

    @staticmethod
    def _deserialize_int8(
        buffer: bytearray,
        context: dict,
        fixed: bool | None = None,
        size: int | None = None,
    ) -> int | list[int]:
        if fixed is not None:
            if fixed is True and size is not None:
                value = struct.unpack_from(f"{size}b", buffer, context["offset"])
                context["offset"] += size
                return list(value)
            elif fixed is False and size is None:
                size = Message._deserialize_uint32(buffer, context)
                return Message._deserialize_int8(buffer, context, fixed=True, size=size)
            else:
                return 0
        else:
            value = struct.unpack_from("b", buffer, context["offset"])[0]
            context["offset"] += 1
            return value

    @staticmethod
    def _deserialize_uint8(
        buffer: bytearray,
        context: dict,
        fixed: bool | None = None,
        size: int | None = None,
    ) -> int | list[int]:
        if fixed is not None:
            if fixed is True and size is not None:
                value = struct.unpack_from(f"{size}B", buffer, context["offset"])
                context["offset"] += size
                return list(value)
            elif fixed is False and size is None:
                size = Message._deserialize_uint32(buffer, context)
                return Message._deserialize_uint8(
                    buffer, context, fixed=True, size=size
                )
            else:
                return 0
        else:
            value = struct.unpack_from("B", buffer, context["offset"])[0]
            context["offset"] += 1
            return value

    @staticmethod
    def _deserialize_char(
        buffer: bytearray,
        context: dict,
        fixed: bool | None = None,
        size: int | None = None,
    ) -> int | list[int]:
        if fixed is not None:
            if fixed is True and size is not None:
                value = struct.unpack_from(f"{size}c", buffer, context["offset"])
                context["offset"] += size
                return list(value)
            elif fixed is False and size is None:
                size = Message._deserialize_uint32(buffer, context)
                return Message._deserialize_char(buffer, context, fixed=True, size=size)
            else:
                return 0
        else:
            value = struct.unpack_from("c", buffer, context["offset"])[0]
            context["offset"] += 1
            return value

    @staticmethod
    def _deserialize_bool(
        buffer: bytearray,
        context: dict,
        fixed: bool | None = None,
        size: int | None = None,
    ) -> int | list[int]:
        if fixed is not None:
            if fixed is True and size is not None:
                value = struct.unpack_from(f"{size}?", buffer, context["offset"])
                context["offset"] += size
                return list(value)
            elif fixed is False and size is None:
                size = Message._deserialize_uint32(buffer, context)
                return Message._deserialize_bool(buffer, context, fixed=True, size=size)
            else:
                return 0
        else:
            value = struct.unpack_from("?", buffer, context["offset"])[0]
            context["offset"] += 1
            return value

    @staticmethod
    def _deserialize_int16(
        buffer: bytearray,
        context: dict,
        fixed: bool | None = None,
        size: int | None = None,
    ) -> int | list[int]:
        if fixed is not None:
            if fixed is True and size is not None:
                value = struct.unpack_from(f"{size}h", buffer, context["offset"])
                context["offset"] += 2 * size
                return list(value)
            elif fixed is False and size is None:
                size = Message._deserialize_uint32(buffer, context)
                return Message._deserialize_int16(
                    buffer, context, fixed=True, size=size
                )
            else:
                return 0
        else:
            value = struct.unpack_from("h", buffer, context["offset"])[0]
            context["offset"] += 2
            return value

    @staticmethod
    def _deserialize_uint16(
        buffer: bytearray,
        context: dict,
        fixed: bool | None = None,
        size: int | None = None,
    ) -> int | list[int]:
        if fixed is not None:
            if fixed is True and size is not None:
                value = struct.unpack_from(f"{size}H", buffer, context["offset"])
                context["offset"] += 2 * size
                return list(value)
            elif fixed is False and size is None:
                size = Message._deserialize_uint32(buffer, context)
                return Message._deserialize_uint16(
                    buffer, context, fixed=True, size=size
                )
            else:
                return 0
        else:
            value = struct.unpack_from("H", buffer, context["offset"])[0]
            context["offset"] += 2
            return value

    @staticmethod
    def _deserialize_int32(
        buffer: bytearray,
        context: dict,
        fixed: bool | None = None,
        size: int | None = None,
    ) -> int | list[int]:
        if fixed is not None:
            if fixed is True and size is not None:
                value = struct.unpack_from(f"{size}i", buffer, context["offset"])
                context["offset"] += 4 * size
                return list(value)
            elif fixed is False and size is None:
                size = Message._deserialize_uint32(buffer, context)
                return Message._deserialize_int32(
                    buffer, context, fixed=True, size=size
                )
            else:
                return 0
        else:
            value = struct.unpack_from("i", buffer, context["offset"])[0]
            context["offset"] += 4
            return value

    @staticmethod
    def _deserialize_uint32(
        buffer: bytearray,
        context: dict,
        fixed: bool | None = None,
        size: int | None = None,
    ) -> int | list[int]:
        if fixed is not None:
            if fixed is True and size is not None:
                value = struct.unpack_from(f"{size}I", buffer, context["offset"])
                context["offset"] += 4 * size
                return list(value)
            elif fixed is False and size is None:
                size = Message._deserialize_uint32(buffer, context)
                return Message._deserialize_uint32(
                    buffer, context, fixed=True, size=size
                )
            else:
                return 0
        else:
            value = struct.unpack_from("I", buffer, context["offset"])[0]
            context["offset"] += 4
            return value

    @staticmethod
    def _deserialize_int64(
        buffer: bytearray,
        context: dict,
        fixed: bool | None = None,
        size: int | None = None,
    ) -> int | list[int]:
        if fixed is not None:
            if fixed is True and size is not None:
                value = struct.unpack_from(f"{size}q", buffer, context["offset"])
                context["offset"] += 8 * size
                return list(value)
            elif fixed is False and size is None:
                size = Message._deserialize_uint32(buffer, context)
                return Message._deserialize_int64(
                    buffer, context, fixed=True, size=size
                )
            else:
                return 0
        else:
            value = struct.unpack_from("q", buffer, context["offset"])[0]
            context["offset"] += 8
            return value

    @staticmethod
    def _deserialize_uint64(
        buffer: bytearray,
        context: dict,
        fixed: bool | None = None,
        size: int | None = None,
    ) -> int | list[int]:
        if fixed is not None:
            if fixed is True and size is not None:
                value = struct.unpack_from(f"{size}Q", buffer, context["offset"])
                context["offset"] += 8 * size
                return list(value)
            elif fixed is False and size is None:
                size = Message._deserialize_uint32(buffer, context)
                return Message._deserialize_uint64(
                    buffer, context, fixed=True, size=size
                )
            else:
                return 0
        else:
            value = struct.unpack_from("Q", buffer, context["offset"])[0]
            context["offset"] += 8
            return value

    @staticmethod
    def _deserialize_float(
        buffer: bytearray,
        context: dict,
        fixed: bool | None = None,
        size: int | None = None,
    ) -> float | list[float]:
        if fixed is not None:
            if fixed is True and size is not None:
                value = struct.unpack_from(f"{size}f", buffer, context["offset"])
                context["offset"] += 4 * size
                return list(value)
            elif fixed is False and size is None:
                size = Message._deserialize_uint32(buffer, context)
                return Message._deserialize_float(
                    buffer, context, fixed=True, size=size
                )
            else:
                return 0
        else:
            value = struct.unpack_from("f", buffer, context["offset"])[0]
            context["offset"] += 4
            return value

    @staticmethod
    def _deserialize_double(
        buffer: bytearray,
        context: dict,
        fixed: bool | None = None,
        size: int | None = None,
    ) -> float | list[float]:
        if fixed is not None:
            if fixed is True and size is not None:
                value = struct.unpack_from(f"{size}d", buffer, context["offset"])
                context["offset"] += 8 * size
                return list(value)
            elif fixed is False and size is None:
                size = Message._deserialize_uint32(buffer, context)
                return Message._deserialize_double(
                    buffer, context, fixed=True, size=size
                )
            else:
                return 0
        else:
            value = struct.unpack_from("d", buffer, context["offset"])[0]
            context["offset"] += 8
            return value

    @staticmethod
    def _deserialize_string(
        buffer: bytearray,
        context: dict,
        fixed: bool | None = None,
        size: int | None = None,
    ) -> str | list[str]:
        if fixed is not None:
            if fixed is True and size is not None:
                value = []
                # print("fixed arr size: " + str(size))
                for _ in range(size):
                    value.append(Message._deserialize_string(buffer, context))
                return list(value)
            elif fixed is False and size is None:
                size = Message._deserialize_uint32(buffer, context)
                # print("vec size:" + str(size))
                return Message._deserialize_string(
                    buffer, context, fixed=True, size=size
                )
            else:
                return 0
        else:
            size = Message._deserialize_uint32(buffer, context)
            # print("string size:" + str(size))
            value = struct.unpack_from(f"{size}s", buffer, context["offset"])[0]
            context["offset"] += size
            return value.decode()

    @staticmethod
    def _deserialize_message(
        buffer: bytearray,
        context: dict,
        obj_type: type,
        fixed: bool | None = None,
        size: int | None = None,
    ) -> object:
        if fixed is not None:
            if fixed is True and size is not None:
                value = []
                for _ in range(size):
                    value.append(
                        Message._deserialize_message(buffer, context, obj_type)
                    )
                return list(value)
            elif fixed is False and size is None:
                size = Message._deserialize_uint32(buffer, context)
                return Message._deserialize_message(
                    buffer, context, obj_type, fixed=True, size=size
                )
            else:
                return 0
        else:
            obj = obj_type()
            obj.deserialize(buffer, context)
            return obj

    @staticmethod
    def _deserialize_map(
        buffer: bytearray,
        context: dict,
        key_func: callable,
        value_func: callable,
        obj_type: type | None = None,
    ) -> dict:
        keys = key_func(buffer, context, False, None)
        if obj_type is None:
            values = value_func(buffer, context, False, None)
        else:
            values = value_func(buffer, context, obj_type, False, None)
        return dict(zip(keys, values))

    @staticmethod
    def _size_int8() -> int:
        return 1

    @staticmethod
    def _size_uint8() -> int:
        return 1

    @staticmethod
    def _size_char() -> int:
        return 1

    @staticmethod
    def _size_bool() -> int:
        return 1

    @staticmethod
    def _size_int16() -> int:
        return 2

    @staticmethod
    def _size_uint16() -> int:
        return 2

    @staticmethod
    def _size_int32() -> int:
        return 4

    @staticmethod
    def _size_uint32() -> int:
        return 4

    @staticmethod
    def _size_int64() -> int:
        return 8

    @staticmethod
    def _size_uint64() -> int:
        return 8

    @staticmethod
    def _size_float() -> int:
        return 4

    @staticmethod
    def _size_double() -> int:
        return 8

    @staticmethod
    def _size_string(value: str) -> int:
        return 4 + len(value)

    @staticmethod
    def _size_message(value: object) -> int:
        return value.size()

    @staticmethod
    def _size_vector_number(value: list, size: int) -> int:
        return 4 + len(value) * size

    @staticmethod
    def _size_vector_string(value: list) -> int:
        return 4 + sum(len(item) for item in value)

    @staticmethod
    def _size_vector_message(value: list) -> int:
        return 4 + sum(item.size() for item in value)

    @staticmethod
    def _size_array_number(value: list, size: int) -> int:
        return len(value) * size

    @staticmethod
    def _size_array_string(value: list, size: int) -> int:
        # return sum(len(item) for item in value)
        return sum(Message._size_string(item) for item in value[:size])

    @staticmethod
    def _size_array_message(value: list, size: int) -> int:
        return sum(Message._size_message(item) for item in value[:size])

    @staticmethod
    def _size_map_number_to_number(value: dict, key_size: int, value_size: int) -> int:
        return 8 + len(value) * (key_size + value_size)

    @staticmethod
    def _size_map_number_to_string(value: dict, key_size: int) -> int:
        return (
            8
            + len(value) * key_size
            + sum(Message._size_string(item) for item in value.values)
        )

    @staticmethod
    def _size_map_number_to_message(value: dict, key_size: int) -> int:
        return (
            8
            + len(value) * key_size
            + sum(Message._size_message(item) for item in value.values)
        )

    @staticmethod
    def _size_map_string_to_number(value: dict, value_size: int) -> int:
        return (
            8
            + sum(Message._size_string(item) for item in value.keys)
            + len(value) * value_size
        )

    @staticmethod
    def _size_map_string_to_string(value: dict) -> int:
        return 8 + sum(
            Message._size_string(k) + Message._size_string(v) for k, v in value.items()
        )

    @staticmethod
    def _size_map_string_to_message(value: dict) -> int:
        return 8 + sum(
            Message._size_string(k) + Message._size_message(v) for k, v in value.items()
        )
