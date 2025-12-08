#!/usr/bin/env python3
"""
Transmission Statistics Calculator

Analyzes communication data from two SQLite databases:
- protocol_communication.db (sender/desktop side)
- turtlebot_communication.db (receiver/robot side)

Calculates:
1. Successful transmission rate
2. False positive rate
3. Error catch rate (CRC)
4. Transmission speed and information throughput
"""

import sqlite3
import sys
from pathlib import Path
from dataclasses import dataclass
from typing import List, Optional, Tuple

# Timestamp tolerance for matching entries (milliseconds)
TIMESTAMP_TOLERANCE_MS = 5000  # 5 seconds window for matching

# Session gap threshold (if gap > this, consider it a new session)
SESSION_GAP_MS = 60000  # 60 seconds


@dataclass
class SentEntry:
    id: int
    start_timestamp: int
    end_timestamp: Optional[int]
    response_time: Optional[int]
    command: str
    command_bit_encoded: int
    confirmation_type: Optional[int]


@dataclass
class ReceivedEntry:
    id: int
    timestamp: int
    command_bit_encoded: int
    crc_valid: bool
    command: str
    confirmation_sent: int


@dataclass
class MatchedPair:
    sent: SentEntry
    received: ReceivedEntry
    timestamp_diff: int  # ms difference


@dataclass
class Session:
    """Represents a continuous test session."""
    id: int
    start_time: int
    end_time: int
    sent_entries: List[SentEntry]
    received_entries: List[ReceivedEntry]


def load_sent_data(db_path: str) -> List[SentEntry]:
    """Load sent data from protocol database."""
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    cursor.execute("""
        SELECT id, startTimeStamp, endTimeStamp, responseTime, command,
               commandBitEncoded, confirmationType
        FROM SentData
        ORDER BY startTimeStamp
    """)
    entries = []
    for row in cursor.fetchall():
        entries.append(SentEntry(
            id=row[0],
            start_timestamp=row[1],
            end_timestamp=row[2] if row[2] else None,
            response_time=row[3] if row[3] else None,
            command=row[4],
            command_bit_encoded=row[5],
            confirmation_type=row[6] if row[6] else None
        ))
    conn.close()
    return entries


def load_received_data(db_path: str) -> List[ReceivedEntry]:
    """Load received data from turtlebot database."""
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    cursor.execute("""
        SELECT id, timeStamp, commandBitEncoded, crc, command, intConfirmationSen
        FROM ReceivedData
        ORDER BY timeStamp
    """)
    entries = []
    for row in cursor.fetchall():
        entries.append(ReceivedEntry(
            id=row[0],
            timestamp=row[1],
            command_bit_encoded=row[2],
            crc_valid=bool(row[3]),
            command=row[4],
            confirmation_sent=row[5]
        ))
    conn.close()
    return entries


def identify_sessions(sent: List[SentEntry], received: List[ReceivedEntry],
                      gap_threshold_ms: int = SESSION_GAP_MS) -> List[Session]:
    """
    Identify distinct test sessions based on time gaps.
    A session is a period of activity without gaps > gap_threshold.
    """
    # Combine all timestamps
    all_events = []
    for s in sent:
        all_events.append(('sent', s.start_timestamp, s))
    for r in received:
        all_events.append(('recv', r.timestamp, r))

    if not all_events:
        return []

    # Sort by timestamp
    all_events.sort(key=lambda x: x[1])

    sessions = []
    current_session_sent = []
    current_session_recv = []
    session_start = all_events[0][1]
    last_timestamp = all_events[0][1]

    for event_type, timestamp, entry in all_events:
        # Check if this is a new session
        if timestamp - last_timestamp > gap_threshold_ms:
            # Save current session if it has any data
            if current_session_sent or current_session_recv:
                sessions.append(Session(
                    id=len(sessions) + 1,
                    start_time=session_start,
                    end_time=last_timestamp,
                    sent_entries=current_session_sent,
                    received_entries=current_session_recv
                ))
            # Start new session
            current_session_sent = []
            current_session_recv = []
            session_start = timestamp

        # Add to current session
        if event_type == 'sent':
            current_session_sent.append(entry)
        else:
            current_session_recv.append(entry)

        last_timestamp = timestamp

    # Don't forget the last session
    if current_session_sent or current_session_recv:
        sessions.append(Session(
            id=len(sessions) + 1,
            start_time=session_start,
            end_time=last_timestamp,
            sent_entries=current_session_sent,
            received_entries=current_session_recv
        ))

    return sessions


def match_entries(sent: List[SentEntry], received: List[ReceivedEntry],
                  tolerance_ms: int = TIMESTAMP_TOLERANCE_MS) -> Tuple[List[MatchedPair], List[SentEntry], List[ReceivedEntry]]:
    """
    Match sent and received entries based on timestamp and command bit encoding.

    Matching strategy:
    - Each sent command can match to at most one received command
    - Each received command can match to at most one sent command
    - Multiple identical commands sent in a row (retries) can each match to separate received commands
    - A false positive is ONLY a received command with NO matching sent command

    Returns:
        - matched_pairs: List of matched (sent, received) pairs
        - unmatched_sent: Sent entries with no matching received entry (dropped)
        - unmatched_received: Received entries with no matching sent entry (true false positives)
    """
    matched_pairs = []
    used_sent_ids = set()
    used_received_ids = set()

    # Sort both lists by timestamp to ensure proper ordering
    sorted_sent = sorted(sent, key=lambda x: x.start_timestamp)
    sorted_received = sorted(received, key=lambda x: x.timestamp)

    # For each received entry, find the best matching sent entry
    # This ensures every received command gets matched if possible
    for recv_entry in sorted_received:
        best_match = None
        best_diff = float('inf')

        for sent_entry in sorted_sent:
            if sent_entry.id in used_sent_ids:
                continue

            # Check if command bit matches
            if sent_entry.command_bit_encoded != recv_entry.command_bit_encoded:
                continue

            # Check timestamp within tolerance
            # Received timestamp should be after sent start timestamp
            time_diff = recv_entry.timestamp - sent_entry.start_timestamp

            if 0 <= time_diff <= tolerance_ms:
                if time_diff < best_diff:
                    best_diff = time_diff
                    best_match = sent_entry

        if best_match:
            matched_pairs.append(MatchedPair(
                sent=best_match,
                received=recv_entry,
                timestamp_diff=best_diff
            ))
            used_sent_ids.add(best_match.id)
            used_received_ids.add(recv_entry.id)

    # Unmatched sent = commands that were sent but never received (dropped)
    unmatched_sent = [s for s in sent if s.id not in used_sent_ids]

    # Unmatched received = commands received with NO matching sent command (true false positives)
    unmatched_received = [r for r in received if r.id not in used_received_ids]

    return matched_pairs, unmatched_sent, unmatched_received


def calculate_statistics(sent: List[SentEntry], received: List[ReceivedEntry],
                         matched: List[MatchedPair], unmatched_sent: List[SentEntry],
                         unmatched_received: List[ReceivedEntry],
                         session_duration_s: Optional[float] = None) -> dict:
    """Calculate all transmission statistics."""

    stats = {}

    # Total counts
    total_sent = len(sent)
    total_received = len(received)
    total_matched = len(matched)
    total_dropped = len(unmatched_sent)
    total_false_positives = len(unmatched_received)

    stats['total_sent'] = total_sent
    stats['total_received'] = total_received
    stats['total_matched'] = total_matched
    stats['total_dropped'] = total_dropped
    stats['total_false_positives'] = total_false_positives

    # 1. Successful Transmission Rate
    # Transmissions that were successfully received AND acted upon
    successful_transmissions = len([m for m in matched if m.received.confirmation_sent == 1])
    stats['successful_transmissions'] = successful_transmissions
    if total_sent > 0:
        stats['success_rate'] = (successful_transmissions / total_sent) * 100
    else:
        stats['success_rate'] = 0.0

    # Also count sent entries that got confirmation (from sender's perspective)
    confirmed_sent = len([s for s in sent if s.confirmation_type == 1])
    stats['confirmed_sent'] = confirmed_sent
    if total_sent > 0:
        stats['sender_confirmed_rate'] = (confirmed_sent / total_sent) * 100
    else:
        stats['sender_confirmed_rate'] = 0.0

    # 2. False Positive Rate
    # Received transmissions not matching any sent transmission
    if total_received > 0:
        stats['false_positive_rate'] = (total_false_positives / total_received) * 100
    else:
        stats['false_positive_rate'] = 0.0

    # Also calculate as percentage of total acted upon
    total_acted = total_matched + total_false_positives
    if total_acted > 0:
        stats['false_positive_of_acted'] = (total_false_positives / total_acted) * 100
    else:
        stats['false_positive_of_acted'] = 0.0

    # 3. Error Catch Rate (CRC)
    crc_failed = len([r for r in received if not r.crc_valid])
    crc_passed = len([r for r in received if r.crc_valid])
    stats['crc_failed'] = crc_failed
    stats['crc_passed'] = crc_passed

    # Error catch rate = CRC rejections / (CRC rejections + false positives that got through)
    # Since CRC failures are silently dropped, they don't appear in the received data
    # We can only track negative confirmations (command decoded but rejected)
    negative_confirmations = len([r for r in received if r.confirmation_sent == 2])
    stats['negative_confirmations'] = negative_confirmations

    # Errors caught = explicit CRC failures + negative confirmations
    errors_caught = crc_failed + negative_confirmations

    # Total transmission attempts that could have errors =
    # errors caught + those that got through incorrectly (false positives)
    total_potential_errors = errors_caught + total_false_positives

    if total_potential_errors > 0:
        stats['error_catch_rate'] = (errors_caught / total_potential_errors) * 100
    elif crc_failed == 0 and negative_confirmations == 0 and total_false_positives == 0:
        # No errors detected at all - either perfect transmission or no erroneous data to catch
        # Report as N/A (represented as -1, displayed specially)
        stats['error_catch_rate'] = -1.0  # Special value for "no errors to evaluate"
    else:
        stats['error_catch_rate'] = 100.0

    # 4. Transmission Speed and Information Throughput
    # Response times from sent entries (time from send start to confirmation)
    response_times = [s.response_time for s in sent if s.response_time and s.response_time > 0]
    if response_times:
        stats['avg_response_time_ms'] = sum(response_times) / len(response_times)
        stats['min_response_time_ms'] = min(response_times)
        stats['max_response_time_ms'] = max(response_times)
    else:
        stats['avg_response_time_ms'] = 0
        stats['min_response_time_ms'] = 0
        stats['max_response_time_ms'] = 0

    # Information throughput
    # Each command is 16 bits (12 data + 4 CRC)
    bits_per_command = 16

    # Use provided session duration or calculate from data
    if session_duration_s is not None and session_duration_s > 0:
        total_time_seconds = session_duration_s
    elif sent:
        first_timestamp = min(s.start_timestamp for s in sent)
        last_timestamp = max(s.end_timestamp for s in sent if s.end_timestamp) if any(s.end_timestamp for s in sent) else max(s.start_timestamp for s in sent)
        total_time_seconds = (last_timestamp - first_timestamp) / 1000.0
    else:
        total_time_seconds = 0

    if total_time_seconds > 0:
        # Bits per second based on all transmissions
        total_bits_sent = total_sent * bits_per_command
        stats['throughput_bps'] = total_bits_sent / total_time_seconds
        stats['effective_throughput_bps'] = (successful_transmissions * bits_per_command) / total_time_seconds
        stats['commands_per_second'] = total_sent / total_time_seconds
    else:
        stats['throughput_bps'] = 0
        stats['effective_throughput_bps'] = 0
        stats['commands_per_second'] = 0

    stats['total_session_time_s'] = total_time_seconds

    # Calculate latency from matched pairs
    latencies = [m.timestamp_diff for m in matched]
    if latencies:
        stats['avg_latency_ms'] = sum(latencies) / len(latencies)
        stats['min_latency_ms'] = min(latencies)
        stats['max_latency_ms'] = max(latencies)
    else:
        stats['avg_latency_ms'] = 0
        stats['min_latency_ms'] = 0
        stats['max_latency_ms'] = 0

    return stats


def print_statistics_table(stats: dict, title: str = "TRANSMISSION STATISTICS REPORT"):
    """Print statistics in a formatted table."""

    print("\n" + "=" * 70)
    print(f"          {title}")
    print("=" * 70)

    # Summary counts
    print("\n┌─────────────────────────────────────────────────────────────────────┐")
    print("│                         SUMMARY COUNTS                              │")
    print("├─────────────────────────────────────────────────────────────────────┤")
    print(f"│ Total Transmissions Sent:              {stats['total_sent']:>10}                   │")
    print(f"│ Total Transmissions Received:          {stats['total_received']:>10}                   │")
    print(f"│ Successfully Matched:                  {stats['total_matched']:>10}                   │")
    print(f"│ Dropped (no match on receiver):        {stats['total_dropped']:>10}                   │")
    print(f"│ False Positives (unmatched received):  {stats['total_false_positives']:>10}                   │")
    print("└─────────────────────────────────────────────────────────────────────┘")

    # Success rates
    print("\n┌─────────────────────────────────────────────────────────────────────┐")
    print("│                    1. SUCCESSFUL TRANSMISSION RATE                  │")
    print("├─────────────────────────────────────────────────────────────────────┤")
    print(f"│ Successful Transmissions:              {stats['successful_transmissions']:>10}                   │")
    print(f"│ Success Rate:                          {stats['success_rate']:>10.2f}%                  │")
    print(f"│ Sender Confirmed Rate:                 {stats['sender_confirmed_rate']:>10.2f}%                  │")
    print("├─────────────────────────────────────────────────────────────────────┤")
    print("│ Note: Success rate = (matched & acted upon) / total sent           │")
    print("│       Sender confirmed = confirmations received by sender          │")
    print("└─────────────────────────────────────────────────────────────────────┘")

    # False positive rate
    print("\n┌─────────────────────────────────────────────────────────────────────┐")
    print("│                      2. FALSE POSITIVE RATE                         │")
    print("├─────────────────────────────────────────────────────────────────────┤")
    print(f"│ Unmatched Received (False Positives):  {stats['total_false_positives']:>10}                   │")
    print(f"│ False Positive Rate (of received):     {stats['false_positive_rate']:>10.2f}%                  │")
    print(f"│ False Positive Rate (of acted):        {stats['false_positive_of_acted']:>10.2f}%                  │")
    print("├─────────────────────────────────────────────────────────────────────┤")
    print("│ Note: Received transmissions not matching any intentional send,    │")
    print("│       or misinterpreted commands that were acted upon              │")
    print("└─────────────────────────────────────────────────────────────────────┘")

    # Error catch rate
    print("\n┌─────────────────────────────────────────────────────────────────────┐")
    print("│                      3. ERROR CATCH RATE (CRC)                      │")
    print("├─────────────────────────────────────────────────────────────────────┤")
    print(f"│ CRC Passed:                            {stats['crc_passed']:>10}                   │")
    print(f"│ CRC Failed (caught by CRC layer):      {stats['crc_failed']:>10}                   │")
    print(f"│ Negative Confirmations Sent:           {stats['negative_confirmations']:>10}                   │")
    if stats['error_catch_rate'] < 0:
        print(f"│ Error Catch Rate:                             N/A                  │")
        print("├─────────────────────────────────────────────────────────────────────┤")
        print("│ Note: No erroneous transmissions detected to evaluate CRC against  │")
    else:
        print(f"│ Error Catch Rate:                      {stats['error_catch_rate']:>10.2f}%                  │")
        print("├─────────────────────────────────────────────────────────────────────┤")
        print("│ Note: Rate of faulty transmissions discarded by CRC layer          │")
    print("└─────────────────────────────────────────────────────────────────────┘")

    # Transmission speed
    print("\n┌─────────────────────────────────────────────────────────────────────┐")
    print("│              4. TRANSMISSION SPEED & THROUGHPUT                     │")
    print("├─────────────────────────────────────────────────────────────────────┤")
    print(f"│ Total Session Time:                    {stats['total_session_time_s']:>10.2f} s                │")
    print(f"│ Commands Per Second:                   {stats['commands_per_second']:>10.4f}                  │")
    print(f"│ Raw Throughput:                        {stats['throughput_bps']:>10.4f} bps               │")
    print(f"│ Effective Throughput:                  {stats['effective_throughput_bps']:>10.4f} bps               │")
    print("├─────────────────────────────────────────────────────────────────────┤")
    print("│                        Response Times                               │")
    print("├─────────────────────────────────────────────────────────────────────┤")
    print(f"│ Average Response Time:                 {stats['avg_response_time_ms']:>10.2f} ms               │")
    print(f"│ Min Response Time:                     {stats['min_response_time_ms']:>10.2f} ms               │")
    print(f"│ Max Response Time:                     {stats['max_response_time_ms']:>10.2f} ms               │")
    print("├─────────────────────────────────────────────────────────────────────┤")
    print("│                      Transmission Latency                           │")
    print("├─────────────────────────────────────────────────────────────────────┤")
    print(f"│ Average Latency:                       {stats['avg_latency_ms']:>10.2f} ms               │")
    print(f"│ Min Latency:                           {stats['min_latency_ms']:>10.2f} ms               │")
    print(f"│ Max Latency:                           {stats['max_latency_ms']:>10.2f} ms               │")
    print("├─────────────────────────────────────────────────────────────────────┤")
    print("│ Note: Response time = send start → confirmation tone received      │")
    print("│       Latency = send start → robot receives command                │")
    print("│       16 bits per command (12 data + 4 CRC)                         │")
    print("└─────────────────────────────────────────────────────────────────────┘")

    print("\n" + "=" * 70)


def print_detailed_matching(matched: List[MatchedPair], unmatched_sent: List[SentEntry],
                           unmatched_received: List[ReceivedEntry]):
    """Print detailed matching information for debugging."""

    print("\n" + "=" * 70)
    print("                    DETAILED MATCHING REPORT")
    print("=" * 70)

    if matched:
        print("\n--- MATCHED PAIRS ---")
        print(f"{'Sent ID':<8} {'Recv ID':<8} {'Command':<30} {'Latency (ms)':<12}")
        print("-" * 60)
        for m in matched:
            print(f"{m.sent.id:<8} {m.received.id:<8} {m.sent.command[:28]:<30} {m.timestamp_diff:<12}")

    if unmatched_sent:
        print("\n--- UNMATCHED SENT (Dropped/Not in Robot DB) ---")
        print(f"{'ID':<8} {'Timestamp':<15} {'Command':<35} {'Confirmation':<12}")
        print("-" * 70)
        for s in unmatched_sent:
            conf = "Yes" if s.confirmation_type == 1 else ("No" if s.confirmation_type == 2 else "None")
            print(f"{s.id:<8} {s.start_timestamp:<15} {s.command[:33]:<35} {conf:<12}")

    if unmatched_received:
        print("\n--- FALSE POSITIVES (received without matching sent) ---")
        print(f"{'ID':<8} {'Timestamp':<15} {'Command':<35} {'CRC Valid':<10}")
        print("-" * 70)
        for r in unmatched_received:
            crc = "Yes" if r.crc_valid else "No"
            print(f"{r.id:<8} {r.timestamp:<15} {r.command[:33]:<35} {crc:<10}")


def print_session_summary(sessions: List[Session]):
    """Print summary of identified sessions."""
    print("\n" + "=" * 70)
    print("                    SESSION IDENTIFICATION")
    print("=" * 70)
    print(f"\nIdentified {len(sessions)} distinct test session(s):\n")

    for session in sessions:
        duration_s = (session.end_time - session.start_time) / 1000.0
        has_both = len(session.sent_entries) > 0 and len(session.received_entries) > 0
        status = "✓ Complete (both sender & receiver data)" if has_both else "✗ Incomplete (missing data)"

        print(f"  Session {session.id}:")
        print(f"    Duration: {duration_s:.2f}s")
        print(f"    Sent entries: {len(session.sent_entries)}")
        print(f"    Received entries: {len(session.received_entries)}")
        print(f"    Status: {status}")
        print()


def generate_csv_output(stats: dict, title: str) -> str:
    """Generate CSV-compatible output for the statistics."""
    lines = [
        f"# {title}",
        "Metric,Value,Unit",
        f"Total Sent,{stats['total_sent']},count",
        f"Total Received,{stats['total_received']},count",
        f"Successfully Matched,{stats['total_matched']},count",
        f"Dropped,{stats['total_dropped']},count",
        f"False Positives,{stats['total_false_positives']},count",
        f"Success Rate,{stats['success_rate']:.2f},%",
        f"Sender Confirmed Rate,{stats['sender_confirmed_rate']:.2f},%",
        f"False Positive Rate (of received),{stats['false_positive_rate']:.2f},%",
        f"False Positive Rate (of acted),{stats['false_positive_of_acted']:.2f},%",
        f"CRC Passed,{stats['crc_passed']},count",
        f"CRC Failed,{stats['crc_failed']},count",
        f"Negative Confirmations,{stats['negative_confirmations']},count",
        f"Error Catch Rate,{stats['error_catch_rate']:.2f},%",
        f"Session Time,{stats['total_session_time_s']:.2f},seconds",
        f"Commands Per Second,{stats['commands_per_second']:.4f},cmd/s",
        f"Raw Throughput,{stats['throughput_bps']:.4f},bps",
        f"Effective Throughput,{stats['effective_throughput_bps']:.4f},bps",
        f"Avg Response Time,{stats['avg_response_time_ms']:.2f},ms",
        f"Min Response Time,{stats['min_response_time_ms']:.2f},ms",
        f"Max Response Time,{stats['max_response_time_ms']:.2f},ms",
        f"Avg Latency,{stats['avg_latency_ms']:.2f},ms",
        f"Min Latency,{stats['min_latency_ms']:.2f},ms",
        f"Max Latency,{stats['max_latency_ms']:.2f},ms",
    ]
    return "\n".join(lines)


def main():
    # Default paths
    script_dir = Path(__file__).parent
    protocol_db = script_dir / "TEST_RESULTS" / "protocol_communication.db"
    turtlebot_db = script_dir / "TEST_RESULTS" / "turtlebot_communication.db"

    # Parse arguments
    output_file = None
    for i, arg in enumerate(sys.argv[1:], 1):
        if arg in ["-o", "--output"] and i < len(sys.argv) - 1:
            output_file = sys.argv[i + 1]

    # Allow command line overrides for database paths
    positional_args = [a for a in sys.argv[1:] if not a.startswith("-") and a != output_file]
    if len(positional_args) >= 2:
        protocol_db = Path(positional_args[0])
        turtlebot_db = Path(positional_args[1])

    # Check if databases exist
    if not protocol_db.exists():
        print(f"Error: Protocol database not found: {protocol_db}")
        sys.exit(1)
    if not turtlebot_db.exists():
        print(f"Error: Turtlebot database not found: {turtlebot_db}")
        sys.exit(1)

    print(f"Loading protocol database: {protocol_db}")
    print(f"Loading turtlebot database: {turtlebot_db}")

    # Load data
    sent_data = load_sent_data(str(protocol_db))
    received_data = load_received_data(str(turtlebot_db))

    print(f"Loaded {len(sent_data)} sent entries")
    print(f"Loaded {len(received_data)} received entries")

    # Identify sessions
    sessions = identify_sessions(sent_data, received_data)
    print_session_summary(sessions)

    # Find sessions with both sender and receiver data (complete sessions)
    complete_sessions = [s for s in sessions if s.sent_entries and s.received_entries]

    csv_output = []

    if complete_sessions:
        print("\n" + "=" * 70)
        print("         STATISTICS FOR COMPLETE SESSIONS ONLY")
        print("=" * 70)

        # Aggregate data from complete sessions
        all_sent = []
        all_received = []
        for session in complete_sessions:
            all_sent.extend(session.sent_entries)
            all_received.extend(session.received_entries)

        # Match and calculate
        matched, unmatched_sent, unmatched_received = match_entries(all_sent, all_received)

        # Calculate total duration of complete sessions
        total_duration = sum((s.end_time - s.start_time) / 1000.0 for s in complete_sessions)

        stats = calculate_statistics(all_sent, all_received, matched, unmatched_sent,
                                     unmatched_received, session_duration_s=total_duration)

        title = f"COMPLETE SESSIONS ({len(complete_sessions)} session(s))"
        print_statistics_table(stats, title)
        csv_output.append(generate_csv_output(stats, title))

        if "--verbose" in sys.argv or "-v" in sys.argv:
            print_detailed_matching(matched, unmatched_sent, unmatched_received)

    # Also show overall statistics
    print("\n" + "=" * 70)
    print("              OVERALL STATISTICS (ALL DATA)")
    print("=" * 70)

    matched, unmatched_sent, unmatched_received = match_entries(sent_data, received_data)
    stats = calculate_statistics(sent_data, received_data, matched, unmatched_sent, unmatched_received)

    title = "ALL SESSIONS (including incomplete)"
    print_statistics_table(stats, title)
    csv_output.append(generate_csv_output(stats, title))

    if "--verbose" in sys.argv or "-v" in sys.argv:
        print_detailed_matching(matched, unmatched_sent, unmatched_received)

    # Save to file if requested
    if output_file:
        with open(output_file, 'w') as f:
            f.write("\n\n".join(csv_output))
        print(f"\nStatistics saved to: {output_file}")


if __name__ == "__main__":
    main()
